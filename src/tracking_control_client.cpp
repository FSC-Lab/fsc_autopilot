#include "tracking_control/tracking_control_client.hpp"

#include <utility>

#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "ros/exceptions.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/TrackingControlConfig.h"
#include "tracking_control/TrackingError.h"
#include "tracking_control/attitude_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/definitions.hpp"
#include "tracking_control/math.hpp"
#include "tracking_control/msg_conversion.hpp"
#include "tracking_control/tracking_controller.hpp"
#include "tracking_control/ude/ude_factory.hpp"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

TrackingControlClient::TrackingControlClient() {
  ros::NodeHandle pnh("~");
  if (!loadParams()) {
    throw ros::InvalidParameterException("Got invalid parameters");
  }
  const ros::Rate outer_rate = pnh.param("tracking_controller/rate", 30);
  const ros::Rate inner_rate = pnh.param("attitude_controller/rate", 250);

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

  cfg_srv_.setCallback([this](auto&& PH1, auto&& PH2) {
    dynamicReconfigureCb(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2));
  });

  outer_loop_ =
      nh_.createTimer(outer_rate, &TrackingControlClient::outerLoop, this);
  inner_loop_ =
      nh_.createTimer(inner_rate, &TrackingControlClient::innerLoop, this);

  watchdog_ =
      nh_.createTimer(outer_rate, &TrackingControlClient::watchdog, this);

  initialized_ = true;
}

void TrackingControlClient::odomCb(const nav_msgs::OdometryConstPtr& msg) {
  // the estimation only provides position measurement
  odom_last_recv_time_ = msg->header.stamp;
  tf2::fromMsg(msg->pose.pose.position, state_.pose.position);
  tf2::fromMsg(msg->pose.pose.orientation, state_.pose.orientation);
  tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
  tf2::fromMsg(msg->twist.twist.angular, state_.twist.angular);
}

void TrackingControlClient::imuCb(const sensor_msgs::ImuConstPtr& msg) {
  imu_last_recv_time_ = msg->header.stamp;
  tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
  tf2::fromMsg(msg->orientation, state_.pose.orientation);
}

void TrackingControlClient::setpointCb(
    const tracking_control::TrackingReferenceConstPtr& msg) {
  tf2::fromMsg(msg->pose.position, outer_ref_.state.pose.position);
  tf2::fromMsg(msg->pose.orientation, outer_ref_.state.pose.orientation);
  tf2::fromMsg(msg->twist.linear, outer_ref_.state.twist.linear);
  tf2::fromMsg(msg->twist.angular, outer_ref_.state.twist.angular);
  tf2::fromMsg(msg->accel.linear, outer_ref_.state.accel.linear);
  tf2::fromMsg(msg->accel.angular, outer_ref_.state.accel.angular);
  outer_ref_.yaw = fsc::deg2rad(msg->yaw);
}

void TrackingControlClient::mavrosStateCb(const mavros_msgs::State& msg) {
  state_last_recv_time_ = msg.header.stamp;
  mavrosState_ = msg;
}

void TrackingControlClient::outerLoop(const ros::TimerEvent& event) {
  // calculate time step
  ude_params_->dt = (event.current_real - event.last_real).toSec();

  if (ude_params_->dt <= 0.0 || ude_params_->dt > 1.0) {
    return;
  }
  // check drone status
  ude_params_->ude_active = (mavrosState_.connected != 0U) &&
                            (mavrosState_.armed != 0U) &&
                            (mavrosState_.mode == "OFFBOARD");

  fsc::TrackingControllerError pos_ctrl_err;
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      tracking_ctrl_.run(state_, outer_ref_, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    ROS_ERROR("Outer controller failed!: %s", outer_success.message().c_str());
    return;
  }

  const auto& [thrust, orientation_sp] = pos_ctrl_out.input.thrust_attitude();

  tracking_control::TrackingError tracking_error_msg;
  tf2::toMsg(tf2::Stamped(pos_ctrl_err, event.current_real, ""),
             tracking_error_msg);
  tracking_error_pub_.publish(tracking_error_msg);
  cmd_.thrust =
      std::clamp(static_cast<float>(motor_curve_.vals(thrust)), 0.0F, 1.0F);
  // define output messages
  if (enable_inner_controller_) {
    inner_ref_.state.pose.orientation = orientation_sp;
  } else {
    cmd_.header.stamp = event.current_real;
    cmd_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                     mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                     mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    cmd_.orientation = tf2::toMsg(orientation_sp);

    setpoint_pub_.publish(cmd_);
  }
  // convert thrust command to normalized thrust input
  // if the thrust is an acc command, then the thrust curve must change as well
}

void TrackingControlClient::innerLoop(const ros::TimerEvent& event) {
  ac_params_->dt = (event.current_real - event.last_real).toSec();
  if (ac_params_->dt <= 0.0 || ac_params_->dt > 1.0) {
    return;
  }
  fsc::NonlinearGeometricControllerError att_ctrl_err;
  const auto& [att_ctrl_out, inner_success] =
      att_ctrl_.run(state_, inner_ref_, &att_ctrl_err);

  if (enable_inner_controller_) {
    cmd_.header.stamp = event.current_real;
    cmd_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    tf2::toMsg(att_ctrl_out.input.thrust_rates().body_rates, cmd_.body_rate);
    setpoint_pub_.publish(cmd_);
  }
}

void TrackingControlClient::watchdog(const ros::TimerEvent& event) {
  auto now = ros::Time::now();
  std::map<std::string, ros::Duration> elapsed_since_last_recv = {
      {"Odometry", now - odom_last_recv_time_},
      {"Imu", now - imu_last_recv_time_},
      {"Mavros State", now - state_last_recv_time_},
  };
  auto threshold = ros::Duration(3);
  for (const auto& [key, value] : elapsed_since_last_recv) {
    if (value > threshold) {
      ROS_ERROR_THROTTLE(3,
                         "Time since %s was last received exceeded %f seconds",
                         key.c_str(), threshold.toSec());
    }
  }
}

bool TrackingControlClient::loadParams() {
  ros::NodeHandle pnh("~");
  // put load parameters into a function
  enable_inner_controller_ =
      pnh.param("tracking_controller/enable_inner_controller", false);
  ROS_INFO("Inner controller is %s",
           (enable_inner_controller_ ? "enabled" : "disabled"));

  pnh.getParam("tracking_controller/check_reconfiguration",
               check_reconfiguration_);

  if (!tc_params_->load(RosParamLoader{"~tracking_controller"}, &logger_)) {
    ROS_FATAL("Failed to load TrackingController parameters");
    return false;
  }

  tracking_ctrl_.params() = tc_params_;

  if (!ude_params_->load(RosParamLoader{"~tracking_controller/de"}, &logger_)) {
    ROS_FATAL("Failed to load UDE parameters");
    return false;
  }

  auto ude = fsc::UDEFactory::Create(ude_params_, &logger_);
  if (!ude) {
    return false;
  }
  tracking_ctrl_.ude() = std::move(ude);

  if (!ac_params_->load(RosParamLoader{"~attitude_controller"}, &logger_)) {
    ROS_FATAL("Failed to load attitude controller parameters");
    return false;
  }
  att_ctrl_.params() = ac_params_;

  const auto mc_coeffs = pnh.param("tracking_controller/motor_curve",
                                   std::vector<double>{0.1, 0.05});
  motor_curve_ = MotorCurveType(Eigen::VectorXd::Map(
      mc_coeffs.data(), static_cast<Eigen::Index>(mc_coeffs.size())));

  // offboard control mode
  if (enable_inner_controller_) {
    ROS_INFO("Inner controller (rate mode) enabled!");
    ROS_INFO_STREAM(ac_params_->toString());
  } else {
    ROS_INFO("Inner controller bypassed! (attitude setpoint mode)");
  }

  ROS_INFO_STREAM(tc_params_->toString());
  return true;
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
  const auto& attitude_tracking = config.groups.attitude_tracking;
  int max_idx;

  const auto use_sqrt_controller_prev = std::exchange(
      ac_params_->use_sqrt_controller, attitude_tracking.use_sqrt_controller);
  const auto enable_rate_feedforward_prev =
      std::exchange(ac_params_->rate_bf_ff_enabled,
                    attitude_tracking.enable_rate_feedforward);

  const auto input_tc_prev =
      std::exchange(ac_params_->input_tc, attitude_tracking.input_tc);
  const Eigen::Vector3d kp_angle_new{attitude_tracking.roll_p,
                                     attitude_tracking.pitch_p,
                                     attitude_tracking.yaw_p};

  const auto max_kp_angle_step =
      (kp_angle_new - ac_params_->kp_angle).cwiseAbs().maxCoeff(&max_idx);
  if (check_reconfiguration_ && max_kp_angle_step > kMaxParamStep) {
    constexpr char const* kAxname[] = {"roll", "pitch", "yaw"};
    ROS_ERROR("Refusing reconfiguration: delta Kp[%s] = %f > %f",
              kAxname[max_idx], max_kp_angle_step, kMaxParamStep);
  }

  const Eigen::Vector3d kp_angle_prev =
      std::exchange(ac_params_->kp_angle, kp_angle_new);

  const auto& position_tracking = tracker.position_tracking;
  const auto apply_pos_err_saturation_prev =
      std::exchange(tc_params_->apply_pos_err_saturation,
                    position_tracking.apply_pos_err_saturation);

  const Eigen::Vector3d k_pos_new{position_tracking.pos_p_x,  //
                                  position_tracking.pos_p_y,  //
                                  position_tracking.pos_p_z};
  const auto max_k_pos_step =
      (k_pos_new - tc_params_->k_pos).cwiseAbs().maxCoeff(&max_idx);
  if (check_reconfiguration_ && max_k_pos_step > kMaxParamStep) {
    ROS_ERROR("Refusing reconfiguration: ΔKp[%d] = %f > %f", max_idx,
              max_k_pos_step, kMaxParamStep);
    return;
  }

  const Eigen::Vector3d k_pos_prev =
      std::exchange(tc_params_->k_pos, k_pos_new);

  const auto& velocity_tracking = tracker.velocity_tracking;
  const auto apply_vel_err_saturation_prev =
      std::exchange(tc_params_->apply_vel_err_saturation,
                    velocity_tracking.apply_vel_err_saturation);

  const Eigen::Vector3d k_vel_new{velocity_tracking.vel_p_x,  //
                                  velocity_tracking.vel_p_y,  //
                                  velocity_tracking.vel_p_z};
  const auto max_k_vel_step =
      (k_vel_new - tc_params_->k_vel).cwiseAbs().maxCoeff(&max_idx);
  if (check_reconfiguration_ && max_k_vel_step > kMaxParamStep) {
    ROS_ERROR("Refusing reconfiguration: ΔKv[%d] = %f > %f", max_idx,
              max_k_vel_step, kMaxParamStep);
    return;
  }
  const Eigen::Vector3d k_vel_prev =
      std::exchange(tc_params_->k_vel, k_vel_new);

  auto ude_height_threshold_prev =
      std::exchange(ude_params_->ude_height_threshold, ude.height_threshold);

  const auto ude_gain_step = std::abs(ude.gain - ude_params_->ude_gain);
  if (check_reconfiguration_ && ude_gain_step > kMaxParamStep) {
    ROS_ERROR("Refusing reconfiguration: Δ ude_gain = %f > %f", ude_gain_step,
              kMaxParamStep);
    return;
  }
  auto ude_gain_prev = std::exchange(ude_params_->ude_gain, ude.gain);

  ROS_INFO_STREAM_THROTTLE(
      1.0,
      std::boolalpha
          << "Dynamical Reconfigure Results:\nEnabled inner controller"
          << enable_inner_controller_prev << " -> " << enable_inner_controller_
          << "\nUsing sqrt controller: " << use_sqrt_controller_prev << " -> "
          << ac_params_->use_sqrt_controller << "\nenable_rate_feedforward"
          << enable_rate_feedforward_prev << " -> "
          << ac_params_->rate_bf_ff_enabled << "\ninput_tc: " << input_tc_prev
          << " -> " << ac_params_->input_tc
          << "\nAttitude Kp: " << kp_angle_prev.transpose().format(matlab_fmt)
          << " -> " << kp_angle_new.transpose().format(matlab_fmt)
          << "\nPosition Error Saturation: " << apply_pos_err_saturation_prev
          << " -> " << tc_params_->apply_pos_err_saturation
          << "\nKp: " << k_pos_prev.transpose().format(matlab_fmt) << " -> "
          << tc_params_->k_pos.transpose().format(matlab_fmt)
          << "\nVelocity Error Saturation: " << apply_vel_err_saturation_prev
          << " -> " << tc_params_->apply_vel_err_saturation
          << "\nKv: " << k_vel_prev.transpose().format(matlab_fmt) << " -> "
          << tc_params_->k_vel.transpose().format(matlab_fmt)
          << "\nUDE "
             "height threshold: "
          << ude_height_threshold_prev << " -> "
          << ude_params_->ude_height_threshold << "\nUDE gain: "
          << ude_gain_prev << " -> " << ude_params_->ude_gain);
}
}  // namespace nodelib
