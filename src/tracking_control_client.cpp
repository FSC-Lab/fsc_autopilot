#include "tracking_control/tracking_control_client.hpp"

#include <unistd.h>

#include <chrono>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <rclcpp/node.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

#include "fmt/core.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/tracking_controller.hpp"

namespace nodelib {
using std::placeholders::_1;

using namespace std::string_literals;  // NOLINT

void validateAndSetVector(TrackingControlClient& self,
                          std::string_view name,
                          const std::vector<double>& src,
                          Eigen::Ref<Eigen::VectorXd> dst) {
  const auto dst_sz = dst.size();
  const auto src_sz = src.size();
  if (src_sz != 3) {
    RCLCPP_ERROR_STREAM(
        self.get_logger(),
        fmt::format(
            "{} must be a length-{} array; Got size {}", name, dst_sz, src_sz));
    return;
  }
  dst = Eigen::VectorXd::Map(src.data(), static_cast<Eigen::Index>(src.size()));
}

TrackingControlClient::TrackingControlClient(const std::string& name)
    : Node(name) {
  constexpr auto kDefaultRate = 30;
  this->declare_parameter("tracking_controller/rate", kDefaultRate);
  constexpr auto kDefaultKPosXY = 1.0;
  constexpr auto kDefaultKPosZ = 10.0;
  this->declare_parameter("tracking_controller/k_pos/x", kDefaultKPosXY);
  this->declare_parameter("tracking_controller/k_pos/y", kDefaultKPosXY);
  this->declare_parameter("tracking_controller/k_pos/z", kDefaultKPosZ);

  constexpr auto kDefaultKVelXY = 1.5;
  constexpr auto kDefaultkVelZ = 3.3;
  this->declare_parameter("tracking_controller/k_vel/x", kDefaultKVelXY);
  this->declare_parameter("tracking_controller/k_vel/y", kDefaultKVelXY);
  this->declare_parameter("tracking_controller/k_vel/z", kDefaultkVelZ);

  this->declare_parameter("tracking_controller/min_z_acc", 0.0);

  constexpr auto kDefaultMaxAcceleration = 20.0;
  this->declare_parameter("tracking_controller/max_z_acc",
                          kDefaultMaxAcceleration);

  constexpr auto kDefaultMaxTiltAngle = 45.0;
  this->declare_parameter("tracking_controller/max_tilt_angle",
                          kDefaultMaxTiltAngle);

  constexpr auto kDefaultAttitudeControllerTau = 0.1;
  this->declare_parameter("attitude_controller/tau",
                          kDefaultAttitudeControllerTau);

  tc_params_.k_pos
      << this->get_parameter("tracking_controller/k_pos/x").as_double(),
      this->get_parameter("tracking_controller/k_pos/y").as_double(),
      this->get_parameter("tracking_controller/k_pos/z").as_double();

  tc_params_.k_vel
      << this->get_parameter("tracking_controller/k_vel/x").as_double(),
      this->get_parameter("tracking_controller/k_vel/y").as_double(),
      this->get_parameter("tracking_controller/k_vel/z").as_double();

  tc_params_.drag_d.diagonal()
      << this->get_parameter("tracking_controller/drag_d/x").as_double(),
      this->get_parameter("tracking_controller/drag_d/y").as_double(),
      this->get_parameter("tracking_controller/drag_d/z").as_double();

  tc_params_.min_z_accel =
      this->get_parameter("tracking_controller/min_z_acc").as_double();
  tc_params_.max_z_accel =
      this->get_parameter("tracking_controller/max_z_acc").as_double();
  tc_params_.max_tilt_ratio = std::tan(details::deg2rad(
      this->get_parameter("tracking_controller/max_tilt_angle").as_double()));

  tracking_ctrl_.params() = tc_params_;

  ac_params_.time_constant =
      this->get_parameter("attitude_controller/tau").as_double();
  att_ctrl_.params() = ac_params_;

  const auto mc_coeffs = this->declare_parameter<std::vector<double>>(
      "tracking_controller/motor_curve", std::vector<double>{0.1, 0.05});
  motor_curve_ = MotorCurveType(Eigen::VectorXd::Map(
      mc_coeffs.data(), static_cast<Eigen::Index>(mc_coeffs.size())));

  const auto mavros_ns = this->declare_parameter("mavros_ns", "/mavros"s);

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      fmt::format("{}/local_position/pose", mavros_ns),
      1,
      std::bind(&TrackingControlClient::poseCb, this, _1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      fmt::format("{}/local_position/velocity_local", mavros_ns),
      1,
      std::bind(&TrackingControlClient::twistCb, this, _1));

  target_sub_ =
      this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
          "tracking_controller/target",
          1,
          std::bind(&TrackingControlClient::setpointCb, this, _1));
  setpoint_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      fmt::format("{}/setpoint_raw/attitude", mavros_ns), 1);

  auto update_rate =
      this->get_parameter("tracking_controller/rate").as_double();
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate), [this] { mainLoop(); });
}

void TrackingControlClient::poseCb(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  tf2::fromMsg(msg->pose.position, state_.position);
  tf2::fromMsg(msg->pose.orientation, att_ctrl_state_.attitude);
}

void TrackingControlClient::twistCb(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
  tf2::fromMsg(msg->twist.linear, state_.velocity);
}

void TrackingControlClient::setpointCb(
    const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr& msg) {
  validateAndSetVector(*this, "Position", msg->positions, refs_.position);
  validateAndSetVector(*this, "Velocity", msg->velocities, refs_.velocity);
  validateAndSetVector(
      *this, "Acceleration", msg->accelerations, refs_.acceleration);
  refs_.yaw = atan2(refs_.velocity.y(), refs_.velocity.x());
}

void TrackingControlClient::mainLoop() {
  const auto& [pos_ctrl_out, pos_ctrl_err] = tracking_ctrl_.run(state_, refs_);

  const auto& [att_ctrl_out, att_ctrl_err] =
      att_ctrl_.run(att_ctrl_state_, {pos_ctrl_out.orientation});

  mavros_msgs::msg::AttitudeTarget pld;
  pld.header.stamp = this->get_clock()->now();
  pld.thrust = std::clamp(
      static_cast<float>(motor_curve_.vals(pos_ctrl_out.thrust)), 0.0F, 1.0F);
  pld.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
  pld.orientation = tf2::toMsg(pos_ctrl_out.orientation);
  tf2::toMsg(att_ctrl_out.body_rate, pld.body_rate);

  setpoint_pub_->publish(pld);
}
}  // namespace nodelib
