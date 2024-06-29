#include "tracking_control/tracking_control_client.hpp"

#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/TrackingControlConfig.h"
#include "tracking_control/TrackingError.h"
#include "tracking_control/definitions.hpp"
#include "tracking_control/math.hpp"
#include "tracking_control/msg_conversion.hpp"
#include "tracking_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/tracking_controller.hpp"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

TrackingControlClient::TrackingControlClient() {
  ros::NodeHandle pnh("~");
  loadParams();
  ros_rate_ = pnh.param("tracking_controller/rate", 30);
  const ros::Rate update_rate(ros_rate_);

  const auto mavros_ns = pnh.param("mavros_ns", "/mavros"s);

  subs_.emplace("odom"s,
                nh_.subscribe("/state_estimator/local_position/odom/UAV0", 1,
                              &TrackingControlClient::odomCb, this));

  subs_.emplace("accel"s, nh_.subscribe("/mavros/imu/data", 1,
                                        &TrackingControlClient::imuCb, this));

  subs_.emplace("target"s,
                nh_.subscribe("tracking_controller/target", 1,
                              &TrackingControlClient::setpointCb, this));

  subs_.emplace("state"s,
                nh_.subscribe("/mavros/state", 1,
                              &TrackingControlClient::mavrosStateCb, this));

  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);

  setpoint_attitude_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_attitude_error", 1);

  tracking_error_pub_ = nh_.advertise<tracking_control::TrackingError>(
      "tracking_controller/output_data", 1);

  // show the parameters
  dispPara();

  cfg_srv_.setCallback([this](auto&& PH1, auto&& PH2) {
    dynamicReconfigureCb(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2));
  });
  // register the main loop callback function here
  timer_ = nh_.createTimer(update_rate, &TrackingControlClient::mainLoop, this);

  initialized_ = true;
}

void TrackingControlClient::odomCb(const nav_msgs::OdometryConstPtr& msg) {
  // the estimation only provides position measurement
  tf2::fromMsg(msg->pose.pose.position, state_.pose.position);
  tf2::fromMsg(msg->pose.pose.orientation, state_.pose.orientation);
  tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
  tf2::fromMsg(msg->twist.twist.angular, state_.twist.angular);
}

void TrackingControlClient::imuCb(const sensor_msgs::ImuConstPtr& msg) {
  tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
  tf2::fromMsg(msg->orientation, state_.pose.orientation);
}

void TrackingControlClient::setpointCb(
    const tracking_control::TrackingReferenceConstPtr& msg) {
  tf2::fromMsg(msg->pose.position, refs_.state.pose.position);
  tf2::fromMsg(msg->pose.orientation, refs_.state.pose.orientation);
  tf2::fromMsg(msg->twist.linear, refs_.state.twist.linear);
  tf2::fromMsg(msg->twist.angular, refs_.state.twist.angular);
  tf2::fromMsg(msg->accel.linear, refs_.state.accel.linear);
  tf2::fromMsg(msg->accel.angular, refs_.state.accel.angular);
  refs_.yaw = fsc::deg2rad(msg->yaw);
}

void TrackingControlClient::mavrosStateCb(const mavros_msgs::State& msg) {
  mavrosState_ = msg;
}

void TrackingControlClient::mainLoop(const ros::TimerEvent& event) {
  // calculate time step
  tc_params_->dt = (event.current_real - event.last_real).toSec();
  if (tc_params_->dt <= 0.0 || tc_params_->dt > 1.0) {
    return;
  }
  // check drone status
  tc_params_->ude_active = (mavrosState_.connected != 0U) &&
                           (mavrosState_.armed != 0U) &&
                           (mavrosState_.mode == "OFFBOARD");

  fsc::TrackingControllerError pos_ctrl_err;
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      tracking_ctrl_.run(state_, refs_, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    ROS_ERROR("Outer controller failed!: %s", outer_success.message().c_str());
    return;
  }

  const auto& [thrust, orientation_sp] = pos_ctrl_out.input.thrust_attitude();
  // define output messages
  mavros_msgs::AttitudeTarget pld;
  pld.header.stamp = event.current_real;
  if (enable_inner_controller_) {
    fsc::Reference inner_ref;
    inner_ref.state.pose.orientation = orientation_sp;
    fsc::NonlinearGeometricControllerError att_ctrl_err;
    const auto& [att_ctrl_out, inner_success] =
        att_ctrl_.run(state_, inner_ref, &att_ctrl_err);

    geometry_msgs::Vector3Stamped pld_att_err;
    pld_att_err.header.stamp = event.current_real;
    pld_att_err.vector.x = fsc::rad2deg(att_ctrl_err.attitude_error.x());
    pld_att_err.vector.y = fsc::rad2deg(att_ctrl_err.attitude_error.y());
    pld_att_err.vector.z = fsc::rad2deg(att_ctrl_err.attitude_error.z());
    setpoint_attitude_error_pub_.publish(pld_att_err);

    pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    tf2::toMsg(att_ctrl_out.input.thrust_rates().body_rates, pld.body_rate);
  } else {
    pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    pld.orientation = tf2::toMsg(orientation_sp);
  }
  // convert thrust command to normalized thrust input
  // if the thrust is an acc command, then the thrust curve must change as well
  pld.thrust =
      std::clamp(static_cast<float>(motor_curve_.vals(thrust)), 0.0F, 1.0F);

  setpoint_pub_.publish(pld);

  tracking_control::TrackingError tracking_error_msg;
  tf2::toMsg(tf2::Stamped(pos_ctrl_err, event.current_real, ""),
             tracking_error_msg);
  tracking_error_pub_.publish(tracking_error_msg);
}

void TrackingControlClient::dispPara() {
  // offboard control mode
  if (enable_inner_controller_) {
    ROS_INFO("Inner controller (rate mode) enabled!");
  } else {
    ROS_INFO("Inner controller bypassed! (attitude setpoint mode)");
  }
  // update rate
  ROS_INFO("update rate: %f", ros_rate_);

  ROS_INFO_STREAM(tc_params_->toString());
}

void TrackingControlClient::loadParams() {
  ros::NodeHandle pnh("~");
  // put load parameters into a function
  enable_inner_controller_ =
      pnh.param("tracking_controller/enable_inner_controller", false);
  ROS_INFO("Inner controller is %s",
           (enable_inner_controller_ ? "enabled" : "disabled"));

  pnh.getParam("tracking_controller/check_reconfiguration",
               check_reconfiguration_);

  constexpr auto kDefaultKPosXY = 1.0;
  constexpr auto kDefaultKPosZ = 10.0;
  tc_params_->k_pos <<  // Force a line break
      pnh.param("tracking_controller/k_pos/x", kDefaultKPosXY),
      pnh.param("tracking_controller/k_pos/y", kDefaultKPosXY),
      pnh.param("tracking_controller/k_pos/z", kDefaultKPosZ);

  constexpr auto kDefaultKVelXY = 1.5;
  constexpr auto kDefaultkVelZ = 3.3;
  tc_params_->k_vel <<  // Force a line break
      pnh.param("tracking_controller/k_vel/x", kDefaultKVelXY),
      pnh.param("tracking_controller/k_vel/y", kDefaultKVelXY),
      pnh.param("tracking_controller/k_vel/z", kDefaultkVelZ);

  tc_params_->ude_lb << pnh.param("tracking_controller/de/lbx",
                                  tc_params_->ude_lb.x()),
      pnh.param("tracking_controller/de/lby", tc_params_->ude_lb.y()),
      pnh.param("tracking_controller/de/lbz", tc_params_->ude_lb.z());

  tc_params_->ude_ub << pnh.param("tracking_controller/de/lbx",
                                  tc_params_->ude_ub.x()),
      pnh.param("tracking_controller/de/lby", tc_params_->ude_ub.y()),
      pnh.param("tracking_controller/de/lbz", tc_params_->ude_ub.z());

  tc_params_->ude_height_threshold =
      pnh.param("tracking_controller/de/height_threshold",
                tc_params_->ude_height_threshold);
  tc_params_->ude_gain =
      pnh.param("tracking_controller/de/gain", tc_params_->ude_gain);

  pnh.getParam("tracking_controller/de/is_velocity_based",
               tc_params_->ude_is_velocity_based);

  if (!pnh.getParam("tracking_controller/vehicle_mass",
                    tc_params_->vehicle_mass)) {
    ROS_FATAL("Vehicle mass unspecified in parameters!");
    std::terminate();
  }

  constexpr auto kDefaultAttitudeControllerTau = 0.1;
  ac_params_.time_constant =
      pnh.param("attitude_controller/tau", kDefaultAttitudeControllerTau);

  att_ctrl_.params() = ac_params_;

  tc_params_->min_thrust = pnh.param("tracking_controller/min_thrust", 0.0);

  tc_params_->max_thrust =
      pnh.param("tracking_controller/max_thrust", tc_params_->max_thrust);

  constexpr auto kDefaultMaxTiltAngle = 45.0;
  tc_params_->max_tilt_angle =
      pnh.param("tracking_controller/max_tilt_angle", kDefaultMaxTiltAngle);

  tracking_ctrl_.params() = tc_params_;

  const auto mc_coeffs = pnh.param("tracking_controller/motor_curve",
                                   std::vector<double>{0.1, 0.05});
  motor_curve_ = MotorCurveType(Eigen::VectorXd::Map(
      mc_coeffs.data(), static_cast<Eigen::Index>(mc_coeffs.size())));
}

void TrackingControlClient::dynamicReconfigureCb(
    const tracking_control::TrackingControlConfig& config,
    [[maybe_unused]] std::uint32_t level) {
  constexpr double kMaxParamStep = 1.0;

  if (!initialized_) {
    return;
  }
  const Eigen::IOFormat matlab_fmt{
      Eigen::StreamPrecision, 0, ",", "\n;", "", "", "[", "]"};

  const auto enable_inner_controller_prev = std::exchange(
      enable_inner_controller_, config.groups.project.enable_inner_controller);
  const auto& tracker = config.groups.tracker;
  const auto& ude = config.groups.ude;
  int max_idx;

  const auto apply_pos_err_saturation_prev =
      std::exchange(tc_params_->apply_pos_err_saturation,
                    tracker.position_tracking.apply_pos_err_saturation);

  const Eigen::Vector3d k_pos_new{tracker.position_tracking.pos_p_x,  //
                                  tracker.position_tracking.pos_p_y,  //
                                  tracker.position_tracking.pos_p_z};
  const auto max_k_pos_step =
      (k_pos_new - tc_params_->k_pos).cwiseAbs().maxCoeff(&max_idx);
  if (check_reconfiguration_ && max_k_pos_step > kMaxParamStep) {
    ROS_ERROR("Refusing reconfiguration: ΔKp[%d] = %f > %f", max_idx,
              max_k_pos_step, kMaxParamStep);
    return;
  }

  const auto apply_vel_err_saturation_prev =
      std::exchange(tc_params_->apply_vel_err_saturation,
                    tracker.velocity_tracking.apply_vel_err_saturation);

  const Eigen::Vector3d k_pos_prev =
      std::exchange(tc_params_->k_pos, k_pos_new);

  const Eigen::Vector3d k_vel_new{tracker.velocity_tracking.vel_p_x,  //
                                  tracker.velocity_tracking.vel_p_y,  //
                                  tracker.velocity_tracking.vel_p_z};
  const auto max_k_vel_step =
      (k_vel_new - tc_params_->k_vel).cwiseAbs().maxCoeff(&max_idx);
  if (check_reconfiguration_ && max_k_vel_step > kMaxParamStep) {
    ROS_ERROR("Refusing reconfiguration: ΔKv[%d] = %f > %f", max_idx,
              max_k_vel_step, kMaxParamStep);
    return;
  }
  const Eigen::Vector3d k_vel_prev =
      std::exchange(tc_params_->k_vel, k_vel_new);

  auto ude_is_velocity_based_prev =
      std::exchange(tc_params_->ude_is_velocity_based, ude.is_velocity_based);
  auto ude_height_threshold_prev =
      std::exchange(tc_params_->ude_height_threshold, ude.height_threshold);

  const auto ude_gain_step = std::abs(ude.gain - tc_params_->ude_gain);
  if (check_reconfiguration_ && ude_gain_step > kMaxParamStep) {
    ROS_ERROR("Refusing reconfiguration: Δ ude_gain = %f > %f", ude_gain_step,
              kMaxParamStep);
    return;
  }
  auto ude_gain_prev = std::exchange(tc_params_->ude_gain, ude.gain);

  ROS_INFO_STREAM_THROTTLE(
      1.0,
      std::boolalpha
          << "Dynamical Reconfigure Results:\nEnabled inner controller"
          << enable_inner_controller_prev << " -> " << enable_inner_controller_
          << "\nPosition Error Saturation: " << apply_pos_err_saturation_prev
          << " -> " << tc_params_->apply_pos_err_saturation
          << "\nKp: " << k_pos_prev.transpose().format(matlab_fmt) << " -> "
          << tc_params_->k_pos.transpose().format(matlab_fmt)
          << "\nVelocity Error Saturation: " << apply_vel_err_saturation_prev
          << " -> " << tc_params_->apply_vel_err_saturation
          << "\nKv: " << k_vel_prev.transpose().format(matlab_fmt) << " -> "
          << tc_params_->k_vel.transpose().format(matlab_fmt)
          << "\nUDE is "
             "velocity based: "
          << ude_is_velocity_based_prev << " -> "
          << tc_params_->ude_is_velocity_based
          << "\nUDE "
             "height threshold: "
          << ude_height_threshold_prev << " -> "
          << tc_params_->ude_height_threshold << "\nUDE gain: " << ude_gain_prev
          << " -> " << tc_params_->ude_gain);
}
}  // namespace nodelib
