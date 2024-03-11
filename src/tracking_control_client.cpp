#include "tracking_control/tracking_control_client.hpp"

#include "Eigen/src/Core/util/Meta.h"
#include "fmt/core.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/tracking_controller.hpp"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

void validateAndSetVector(std::string_view name,
                          const std::vector<double>& src,
                          Eigen::Ref<Eigen::VectorXd> dst) {
  const auto dst_sz = dst.size();
  const auto src_sz = src.size();
  if (src_sz != 3) {
    ROS_ERROR_STREAM(fmt::format(
        "{} must be a length-{} array; Got size {}", name, dst_sz, src_sz));
    return;
  }
  dst = Eigen::VectorXd::Map(src.data(), static_cast<Eigen::Index>(src.size()));
}

TrackingControlClient::TrackingControlClient() {
  ros::NodeHandle pnh("~");

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

  subs_.emplace("pose"s,
                nh_.subscribe(fmt::format("{}/local_position/pose", mavros_ns),
                              1,
                              &TrackingControlClient::poseCb,
                              this));
  subs_.emplace(
      "twist"s,
      nh_.subscribe(fmt::format("{}/local_position/velocity_local", mavros_ns),
                    1,
                    &TrackingControlClient::twistCb,
                    this));

  subs_.emplace("target"s,
                nh_.subscribe("tracking_controller/target",
                              1,
                              &TrackingControlClient::setpointCb,
                              this));
  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      fmt::format("{}/setpoint_raw/attitude", mavros_ns), 1);
  timer_ = nh_.createTimer(update_rate, &TrackingControlClient::mainLoop, this);
}

void TrackingControlClient::poseCb(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  tf2::fromMsg(msg->pose.position, state_.position);
  tf2::fromMsg(msg->pose.orientation, att_ctrl_state_.attitude);
}

void TrackingControlClient::twistCb(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  tf2::fromMsg(msg->twist.linear, state_.velocity);
}

void TrackingControlClient::setpointCb(
    const trajectory_msgs::JointTrajectoryPointConstPtr& msg) {
  validateAndSetVector("Position", msg->positions, refs_.position);
  validateAndSetVector("Velocity", msg->velocities, refs_.velocity);
  validateAndSetVector("Acceleration", msg->accelerations, refs_.acceleration);
  refs_.yaw = atan2(refs_.velocity.y(), refs_.velocity.x());
}

void TrackingControlClient::mainLoop(const ros::TimerEvent& event) {
  const auto& [pos_ctrl_out, pos_ctrl_err] = tracking_ctrl_.run(state_, refs_);

  const auto& [att_ctrl_out, att_ctrl_err] =
      att_ctrl_.run(att_ctrl_state_, {pos_ctrl_out.orientation});

  mavros_msgs::AttitudeTarget pld;
  pld.header.stamp = event.current_real;
  pld.thrust = std::clamp(
      static_cast<float>(motor_curve_.vals(pos_ctrl_out.thrust)), 0.0F, 1.0F);
  pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  pld.orientation = tf2::toMsg(pos_ctrl_out.orientation);
  tf2::toMsg(att_ctrl_out.body_rate, pld.body_rate);

  setpoint_pub_.publish(pld);
}
}  // namespace nodelib
