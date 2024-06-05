#include "tracking_control/tracking_control_client.hpp"

#include "Eigen/src/Core/util/Meta.h"
#include "fmt/core.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/tracking_controller.hpp"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

void validateAndSetVector(std::string_view name, const std::vector<double> &src,
                          Eigen::Ref<Eigen::VectorXd> dst) {
  const auto dst_sz = dst.size();
  const auto src_sz = src.size();
  if (src_sz != 3) {
    ROS_ERROR_STREAM(fmt::format("{} must be a length-{} array; Got size {}",
                                 name, dst_sz, src_sz));
    return;
  }
  dst = Eigen::VectorXd::Map(src.data(), static_cast<Eigen::Index>(src.size()));
}

TrackingControlClient::TrackingControlClient() {
  ros::NodeHandle pnh("~");

  enable_inner_controller_ =
      pnh.param("tracking_controller/enable_inner_controller", false);
  ROS_INFO("Inner controller is %s",
           (enable_inner_controller_ ? "enabled" : "disabled"));

  const ros::Rate update_rate(pnh.param("tracking_controller/rate", 30));
  constexpr auto kDefaultKPosXY = 1.0;
  constexpr auto kDefaultKPosZ = 10.0;
  tc_params_.k_pos <<  // Force a line break
      pnh.param("tracking_controller/k_pos/x", kDefaultKPosXY),
      pnh.param("tracking_controller/k_pos/y", kDefaultKPosXY),
      pnh.param("tracking_controller/k_pos/z", kDefaultKPosZ);

  constexpr auto kDefaultKVelXY = 1.5;
  constexpr auto kDefaultkVelZ = 3.3;
  tc_params_.k_vel <<  // Force a line break
      pnh.param("tracking_controller/k_vel/x", kDefaultKVelXY),
      pnh.param("tracking_controller/k_vel/y", kDefaultKVelXY),
      pnh.param("tracking_controller/k_vel/z", kDefaultkVelZ);

  tc_params_.drag_d.diagonal()
      << pnh.param("tracking_controller/drag_d/x", 0.0),
      pnh.param("tracking_controller/drag_d/y", 0.0),
      pnh.param("tracking_controller/drag_d/z", 0.0);

  tc_params_.de_lb << pnh.param("tracking_controller/de/lbx",
                                tc_params_.de_lb.x()),
      pnh.param("tracking_controller/de/lby", tc_params_.de_lb.y()),
      pnh.param("tracking_controller/de/lbz", tc_params_.de_lb.z());

  tc_params_.de_ub << pnh.param("tracking_controller/de/lbx",
                                tc_params_.de_ub.x()),
      pnh.param("tracking_controller/de/lby", tc_params_.de_ub.y()),
      pnh.param("tracking_controller/de/lbz", tc_params_.de_ub.z());

  tc_params_.de_height_threshold =
      pnh.param("tracking_controller/de/height_threshold",
                tc_params_.de_height_threshold);
  tc_params_.de_gain =
      pnh.param("tracking_controller/de/gain", tc_params_.de_gain);

  if (!pnh.getParam("tracking_controller/vehicle_mass",
                    tc_params_.vehicle_mass)) {
    ROS_FATAL("Vehicle mass unspecified in parameters!");
    std::terminate();
  }

  ROS_INFO_STREAM(
      ::fmt::format("Tracking controller gains are {}", tc_params_));
  constexpr auto kDefaultAttitudeControllerTau = 0.1;
  ac_params_.time_constant =
      pnh.param("attitude_controller/tau", kDefaultAttitudeControllerTau);

  att_ctrl_.params() = ac_params_;

  tc_params_.min_z_accel = pnh.param("tracking_controller/min_acc", 0.0);

  constexpr auto kDefaultMaxAcceleration = 20.0;
  tc_params_.max_z_accel =
      pnh.param("tracking_controller/max_acc", kDefaultMaxAcceleration);

  constexpr auto kDefaultMaxTiltAngle = 45.0;
  tc_params_.max_tilt_ratio = std::tan(::details::deg2rad(
      pnh.param("tracking_controller/max_tilt_angle", kDefaultMaxTiltAngle)));

  tracking_ctrl_.params() = tc_params_;

  const auto mc_coeffs = pnh.param("tracking_controller/motor_curve",
                                   std::vector<double>{0.1, 0.05});
  motor_curve_ = MotorCurveType(Eigen::VectorXd::Map(
      mc_coeffs.data(), static_cast<Eigen::Index>(mc_coeffs.size())));

  const auto mavros_ns = pnh.param("mavros_ns", "/mavros"s);

  subs_.emplace(
      "pose"s,
      nh_.subscribe(fmt::format("{}/local_position/adjusted", mavros_ns), 1,
                    &TrackingControlClient::poseCb, this));
  subs_.emplace(
      "twist"s,
      nh_.subscribe(fmt::format("{}/local_position/velocity_local", mavros_ns),
                    1, &TrackingControlClient::twistCb, this));

  subs_.emplace("accel"s,
                nh_.subscribe(fmt::format("{}/imu/data", mavros_ns), 1,
                              &TrackingControlClient::imuCb, this));

  subs_.emplace("target"s,
                nh_.subscribe("tracking_controller/target", 1,
                              &TrackingControlClient::setpointCb, this));
  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      fmt::format("{}/setpoint_raw/attitude", mavros_ns), 1);
  setpoint_pos_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_pos_error", 1);
  setpoint_vel_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_vel_error", 1);
  setpoint_attitude_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_attitude_error", 1);

  timer_ = nh_.createTimer(update_rate, &TrackingControlClient::mainLoop, this);
}

void TrackingControlClient::poseCb(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  tf2::fromMsg(msg->pose.position, state_.position);
  tf2::fromMsg(msg->pose.orientation, state_.orientation);
  tf2::fromMsg(msg->pose.orientation, att_ctrl_state_.attitude);
}

void TrackingControlClient::twistCb(
    const geometry_msgs::TwistStampedConstPtr &msg) {
  tf2::fromMsg(msg->twist.linear, state_.velocity);
}

void TrackingControlClient::imuCb(const sensor_msgs::ImuConstPtr &msg) {
  tf2::fromMsg(msg->linear_acceleration, state_.acceleration);
}

void TrackingControlClient::setpointCb(
    const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {
  validateAndSetVector("Position", msg->positions, refs_.position);
  validateAndSetVector("Velocity", msg->velocities, refs_.velocity);
  validateAndSetVector("Acceleration", msg->accelerations, refs_.acceleration);
  refs_.yaw = atan2(refs_.velocity.y(), refs_.velocity.x());
}

void TrackingControlClient::mainLoop(const ros::TimerEvent &event) {
  const auto &[outer_success, pos_ctrl_out, pos_ctrl_err] =
      tracking_ctrl_.run(state_, refs_);

  if (!outer_success) {
    ROS_ERROR("Outer controller failed!");
    return;
  }

  mavros_msgs::AttitudeTarget pld;
  pld.header.stamp = event.current_real;
  pld.thrust = std::clamp(
      static_cast<float>(motor_curve_.vals(pos_ctrl_out.thrust)), 0.0F, 1.0F);

  if (enable_inner_controller_) {
    ROS_DEBUG("Running inner controller");
    const auto &[inner_success, att_ctrl_out, att_ctrl_err] =
        att_ctrl_.run(att_ctrl_state_, {pos_ctrl_out.orientation});

    pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    tf2::toMsg(att_ctrl_out.body_rate, pld.body_rate);
  } else {
    pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE &
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE &
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    pld.orientation = tf2::toMsg(pos_ctrl_out.orientation);
  }

  geometry_msgs::Vector3Stamped pld_pos_err;
  pld_pos_err.header.stamp = event.current_real;
  tf2::toMsg(pos_ctrl_err.position_error, pld_pos_err.vector);

  geometry_msgs::Vector3Stamped pld_vel_err;
  pld_vel_err.header.stamp = event.current_real;
  tf2::toMsg(pos_ctrl_err.velocity_error, pld_vel_err.vector);

  geometry_msgs::Vector3Stamped pld_att_err;
  pld_att_err.header.stamp = event.current_real;

  // convert radians to degrees
  pld_att_err.vector.x = att_ctrl_err.attitude_error.x() * 180.0 / M_PI;
  pld_att_err.vector.y = att_ctrl_err.attitude_error.y() * 180.0 / M_PI;
  pld_att_err.vector.z = att_ctrl_err.attitude_error.z() * 180.0 / M_PI;

  setpoint_pub_.publish(pld);
  setpoint_pos_error_pub_.publish(pld_pos_err);
  setpoint_vel_error_pub_.publish(pld_vel_err);
  setpoint_attitude_error_pub_.publish(pld_att_err);
}
}  // namespace nodelib
