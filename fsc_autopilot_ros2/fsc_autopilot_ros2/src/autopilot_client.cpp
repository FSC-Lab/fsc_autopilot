// Copyright © 2024 FSC Lab
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "fsc_autopilot_ros2/autopilot_client.hpp"

#include <rclcpp/exceptions/exceptions.hpp>
#include <utility>

#include "fsc_autopilot/attitude_control/attitude_control_error.hpp"
#include "fsc_autopilot/attitude_control/nonlinear_geometric_controller.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/math/math_extras.hpp"
#include "fsc_autopilot/position_control/tracking_controller.hpp"
#include "fsc_autopilot_msgs/msg/tracking_error.hpp"
#include "fsc_autopilot_ros2/msg_conversion.hpp"
#include "fsc_autopilot_ros2/ros2_support.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

TrackingControlClient::TrackingControlClient()
    : rclcpp::Node("tracking_control_clieng") {
  using namespace std::placeholders;
  if (!loadParams()) {
    throw rclcpp::exceptions::InvalidParametersException(
        "Got invalid parameters");
  }
  this->declare_parameter<double>("position_controller/rate", 30.0);
  this->declare_parameter("attitude_controller/rate", 250);

  double outer_rate;
  this->get_parameter("position_controller/rate", outer_rate);
  double inner_rate;
  this->get_parameter("attitude_controller/rate", inner_rate);
  this->declare_parameter("mavros_ns", "/mavros"s);
  std::string mavros_ns;
  this->get_parameter("mavros_ns", mavros_ns);

  subs_.emplace("odom"s,
                this->create_subscription<nav_msgs::msg::Odometry>(
                    "/state_estimator/local_position/odom/UAV0",
                    1,
                    std::bind(&TrackingControlClient::odomCb, this, _1)));

  subs_.emplace("accel"s,
                this->create_subscription<sensor_msgs::msg::Imu>(
                    "/mavros/imu/data",
                    1,
                    std::bind(&TrackingControlClient::imuCb, this, _1)));

  subs_.emplace(
      "target"s,
      this->create_subscription<fsc_autopilot_msgs::msg::TrackingReference>(
          "position_controller/target",
          1,
          std::bind(&TrackingControlClient::setpointCb, this, _1)));

  subs_.emplace(
      "state"s,
      this->create_subscription<mavros_msgs::msg::State>(
          "/mavros/state",
          1,
          std::bind(&TrackingControlClient::mavrosStateCb, this, _1)));

  setpoint_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);

  attitude_error_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "attitude_controller/output_data", 1);

  tracking_error_pub_ =
      this->create_publisher<fsc_autopilot_msgs::msg::TrackingError>(
          "position_controller/output_data", 1);

  outer_loop_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / outer_rate),
      std::bind(&TrackingControlClient::outerLoop, this));
  inner_loop_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / inner_rate),
      std::bind(&TrackingControlClient::innerLoop, this));

  watchdog_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / outer_rate),
      std::bind(&TrackingControlClient::watchdog, this));

  initialized_ = true;
}

void TrackingControlClient::odomCb(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  // the estimation only provides position measurement
  odom_last_recv_time_ = msg->header.stamp;
  tf2::fromMsg(msg->pose.pose.position, state_.pose.position);
  tf2::fromMsg(msg->pose.pose.orientation, state_.pose.orientation);
  tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
  tf2::fromMsg(msg->twist.twist.angular, state_.twist.angular);
}

void TrackingControlClient::imuCb(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  imu_last_recv_time_ = msg->header.stamp;
  tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
  tf2::fromMsg(msg->orientation, state_.pose.orientation);
}

void TrackingControlClient::setpointCb(
    const fsc_autopilot_msgs::msg::TrackingReference::ConstSharedPtr& msg) {
  tf2::fromMsg(msg->pose.position, outer_ref_.state.pose.position);
  tf2::fromMsg(msg->pose.orientation, outer_ref_.state.pose.orientation);
  tf2::fromMsg(msg->twist.linear, outer_ref_.state.twist.linear);
  tf2::fromMsg(msg->twist.angular, outer_ref_.state.twist.angular);
  tf2::fromMsg(msg->accel.linear, outer_ref_.state.accel.linear);
  tf2::fromMsg(msg->accel.angular, outer_ref_.state.accel.angular);
  outer_ref_.yaw = fsc::deg2rad(msg->yaw);
}

void TrackingControlClient::mavrosStateCb(const mavros_msgs::msg::State& msg) {
  state_last_recv_time_ = msg.header.stamp;
  mavrosState_ = msg;
}

void TrackingControlClient::outerLoop() {
  // calculate time step
  auto current_real = get_clock()->now();
  const auto dt = (current_real - outer_loop_event_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // check drone status
  tracking_ctrl_.toggleIntegration(mavrosState_.connected &&
                                   mavrosState_.armed &&
                                   (mavrosState_.mode == "OFFBOARD"));

  fsc::TrackingControllerError pos_ctrl_err;
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      tracking_ctrl_.run(state_, outer_ref_, dt, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    logger_.log(fsc::Severity::kError)
        << "Outer controller failed!: " << outer_success.message();
    return;
  }

  const auto& [thrust, orientation_sp] = pos_ctrl_out.input.thrust_attitude();

  fsc_autopilot_msgs::msg::TrackingError tracking_error_msg;
  tf2::toMsg(tf2::Stamped(pos_ctrl_err, tf2_ros::fromRclcpp(current_real), ""),
             tracking_error_msg);
  tracking_error_pub_->publish(tracking_error_msg);
  cmd_.thrust =
      std::clamp(static_cast<float>(motor_curve_.vals(thrust)), 0.0F, 1.0F);
  // define output messages
  if (enable_inner_controller_) {
    inner_ref_.state.pose.orientation = orientation_sp;
  } else {
    cmd_.header.stamp = current_real;
    cmd_.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                     mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                     mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    cmd_.orientation = tf2::toMsg(orientation_sp);

    setpoint_pub_->publish(cmd_);
  }
  // convert thrust command to normalized thrust input
  // if the thrust is an acc command, then the thrust curve must change as well
}

void TrackingControlClient::innerLoop() {
  auto current_real = get_clock()->now();
  if (inner_loop_event_time_.nanoseconds() == 0) {
    inner_loop_event_time_ = current_real;
    return;
  }
  const auto dt = (current_real - inner_loop_event_time_).seconds();
  inner_loop_event_time_ = current_real;

  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  fsc::AttitudeControlError att_ctrl_err;
  const auto& [att_ctrl_out, inner_success] =
      att_ctrl_.run(state_, inner_ref_, dt, &att_ctrl_err);

  if (enable_inner_controller_) {
    cmd_.header.stamp = current_real;
    cmd_.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
    tf2::toMsg(att_ctrl_out.input.thrust_rates().body_rates, cmd_.body_rate);
    setpoint_pub_->publish(cmd_);

    geometry_msgs::msg::Vector3Stamped attitude_error_msg;
    attitude_error_msg.header.stamp = current_real;
    tf2::toMsg(att_ctrl_err.attitude_error, attitude_error_msg.vector);
    attitude_error_pub_->publish(attitude_error_msg);
  }
}

void TrackingControlClient::watchdog() {
  auto now = this->get_clock()->now();
  std::map<std::string, rclcpp::Duration> elapsed_since_last_recv = {
      {"Odometry", now - odom_last_recv_time_},
      {"Imu", now - imu_last_recv_time_},
      {"Mavros2 State", now - state_last_recv_time_},
  };
  auto threshold = rclcpp::Duration::from_seconds(3);
  for (const auto& [key, value] : elapsed_since_last_recv) {
    if (value > threshold) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3,
          "Time since %s was last received exceeded %f seconds",
          key.c_str(),
          threshold.seconds());
    }
  }
}

bool TrackingControlClient::loadParams() {
  // Position controller parameters
  if (!tc_params_.load(
          RosParamLoader{this->create_sub_node("position_controller")},
          &logger_)) {
    logger_.log(fsc::Severity::kError,
                "Failed to load TrackingController parameters");
    return false;
  }

  if (!tracking_ctrl_.setParams(tc_params_, &logger_)) {
    logger_.log(fsc::Severity::kError,
                "Failed to set tracking controller parameters");
    return false;
  }

  // Attitude controller parameters
  if (!ac_params_.load(
          RosParamLoader{this->create_sub_node("attitude_controller")},
          &logger_)) {
    logger_.log(fsc::Severity::kError,
                "Failed to load attitude controller parameters");
    return false;
  }
  if (!att_ctrl_.setParams(ac_params_, &logger_)) {
    logger_.log(fsc::Severity::kError,
                "Failed to set attitude controller parameters");
    return false;
  }

  declare_parameter("enable_inner_controller", false);
  declare_parameter("check_reconfiguration", false);

  declare_parameter<std::vector<double>>("motor_curve");
  // Core client lib parameters
  // put load parameters into a function
  get_parameter("enable_inner_controller", enable_inner_controller_);

  get_parameter("check_reconfiguration", check_reconfiguration_);

  std::vector<double> mc_coeffs;
  if (!get_parameter("motor_curve", mc_coeffs)) {
    logger_.log(fsc::Severity::kError,
                "Failed to get motor model coefficients!");
  }
  motor_curve_ = MotorCurveType(Eigen::VectorXd::Map(
      mc_coeffs.data(), static_cast<Eigen::Index>(mc_coeffs.size())));

  // offboard control mode
  if (enable_inner_controller_) {
    logger_.log(fsc::Severity::kInfo, "Inner controller (rate mode) enabled!");
    logger_.log(fsc::Severity::kInfo) << ac_params_.toString();
  } else {
    logger_.log(fsc::Severity::kInfo,
                "Inner controller bypassed! (attitude setpoint mode)");
  }

  logger_.log(fsc::Severity::kInfo) << tc_params_.toString();
  return true;
}

}  // namespace nodelib
