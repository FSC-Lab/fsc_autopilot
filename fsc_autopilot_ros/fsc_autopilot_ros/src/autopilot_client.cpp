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

#include "fsc_autopilot_ros/autopilot_client.hpp"

#include <fsc_autopilot/core/controller_base.hpp>
#include <memory>
#include <utility>

#include "fsc_autopilot/attitude_control/apm_attitude_controller.hpp"
#include "fsc_autopilot/attitude_control/attitude_controller_factory.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/core/vehicle_model.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"
#include "fsc_autopilot/position_control/position_controller_factory.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot/ude/ude_factory.hpp"
#include "fsc_autopilot_msgs/msg/attitude_controller_state.hpp"
#include "fsc_autopilot_msgs/msg/position_controller_reference.hpp"
#include "fsc_autopilot_msgs/msg/position_controller_state.hpp"
#include "fsc_autopilot_msgs/msg/ude_state.hpp"
#include "fsc_autopilot_ros/msg_conversion.hpp"
#include "fsc_autopilot_ros/ros_support.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/exceptions.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"

using namespace std::string_literals;  // NOLINT

namespace details {

bool setupPositionController(std::unique_ptr<fsc::PositionControllerBase>& ctl,
                             double& rate, RosLogger& logger) {
  auto node = rclcpp::Node::make_shared("_"); // Dummy node for parameter loading
  const auto type = node->declare_parameter("position_controller.type", "robust_controller"s);
  if (type.empty()) {
    throw rclcpp::exceptions::InvalidParameterValueException(
        "Position controller type cannot be empty");
  }

  ctl = fsc::PositionControllerFactory::Create(type, logger);

  auto params = ctl->getParams(true);
  if (!params->load(node, "position_controller.", logger)) {
    RCLCPP_FATAL(logger.get_logger(), "Failed to load position controller parameters");
    return false;
  }
  if (!ctl->setParams(*params, logger)) {
    RCLCPP_FATAL(logger.get_logger(), "Failed to set position controller parameters");
    return false;
  }
  rate = node->declare_parameter("position_controller.rate", rate);
  RCLCPP_INFO_STREAM(logger.get_logger(), params->toString());
  return true;
}

std::unique_ptr<fsc::UDEBase> setupUDE(RosLogger& logger) {
  auto node = rclcpp::Node::make_shared("_"); // Dummy node for parameter loading
  const auto type = node->declare_parameter("ude.type", "velocity_based"s);

  if (type.empty()) {
    return nullptr;
  }
  auto res = fsc::UDEFactory::Create(type, logger);
  fsc::UDEParameters params;
  if (!params.load(node, "ude.", logger) || !res->setParams(params, logger)) {
    RCLCPP_ERROR(logger.get_logger(), "Got invalid parameters");
    return nullptr;
  }
  RCLCPP_INFO_STREAM(logger.get_logger(), params.toString());
  return res;
}

enum class AttitudeControllerSetupStatus { kSuccess, kFailed, kRejected };

AttitudeControllerSetupStatus setupAttitudeController(
    std::unique_ptr<fsc::AttitudeControllerBase>& ctl, double& rate,
    RosLogger& logger) {
  auto node = rclcpp::Node::make_shared("_"); // Dummy node for parameter loading
  const auto type = node->declare_parameter("attitude_controller.type", "simple"s);
  if (type.empty()) {
    return AttitudeControllerSetupStatus::kRejected;
  }
  ctl = fsc::AttitudeControllerFactory::Create(type, logger);

  auto params = ctl->getParams(true);
  if (!params->load(node, "attitude_controller.", logger)) {
    RCLCPP_FATAL(logger.get_logger(), "Failed to load attitude controller parameters");
    return AttitudeControllerSetupStatus::kFailed;
  }

  if (!ctl->setParams(*params, logger)) {
    RCLCPP_FATAL(logger.get_logger(), "Failed to set attitude controller parameters");
    return AttitudeControllerSetupStatus::kFailed;
  }
  rate = node->declare_parameter("attitude_controller.rate", rate);

  RCLCPP_INFO_STREAM(logger.get_logger(), params->toString());
  return AttitudeControllerSetupStatus::kSuccess;
}

bool CheckSensorAge(const std::string& type, rclcpp::Time now, rclcpp::Time then,
                    double period, rclcpp::Logger& logger) {
  constexpr int kMeasurementStaleFactor = 5;

  if (then == rclcpp::Time{0, 0, RCL_ROS_TIME}) {
    RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 1000, 
                         "First %s measurement not yet received", type.c_str());
    return false;
  }

  const auto imu_data_age = (now - then).seconds();
  if (imu_data_age > kMeasurementStaleFactor * period) {
    RCLCPP_WARN_THROTTLE(
        logger, *node->get_clock(), 1000,
        "Age of the last %s measurement is %fs > %dx control period %fs",
        type.c_str(), imu_data_age, kMeasurementStaleFactor, period);
  }
  return true;
}

}  // namespace details

static constexpr double kDefaultPositionControllerRate = 30.0;
static constexpr double kDefaultAttitudeControllerRate = 250.0;

AutopilotClient::AutopilotClient(rclcpp::Node::SharedPtr node) 
  : node_(node), logger_(node->get_logger()) {
  
  if (fsc::VehicleModelParameters params;
      !params.load(node, "vehicle.", logger_) ||
      !mdl_.setParams(params, logger_)) {
    throw rclcpp::exceptions::InvalidParameterValueException(
        "Failed to get required vehicle model parameters");
  }

  // Call out the type of the vehicle to the user
  const std::string ruler(80, '#');
  RCLCPP_INFO(logger_, "\033[1;32m\n%s\n    Starting autopilot for vehicle: %-45s\n%s",
           ruler.c_str(), mdl_.vehicle_name.c_str(), ruler.c_str());

  auto outer_rate = kDefaultPositionControllerRate;
  if (!details::setupPositionController(pos_ctrl_, outer_rate, logger_)) {
    throw std::runtime_error("Failed to setup position controller");
  }
  outer_period_ = 1.0 / outer_rate;
  outer_loop_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / outer_rate),
      std::bind(&AutopilotClient::outerLoop, this));
  RCLCPP_INFO(logger_, "Running position controller at %f hz", outer_rate);

  ude_ = details::setupUDE(logger_);

  auto inner_rate = kDefaultAttitudeControllerRate;
  switch (details::setupAttitudeController(att_ctrl_, inner_rate, logger_)) {
    case details::AttitudeControllerSetupStatus::kSuccess:
      inner_period_ = 1.0 / inner_rate;
      inner_loop_ = node_->create_wall_timer(
          std::chrono::duration<double>(1.0 / inner_rate),
          std::bind(&AutopilotClient::innerLoop, this));

      enable_inner_controller_ = node_->declare_parameter("enable_inner_controller", true);
      RCLCPP_INFO(logger_, "Running attitude controller at %f hz; Initially %sabled",
               inner_rate, enable_inner_controller_ ? "en" : "dis");

      break;
    case details::AttitudeControllerSetupStatus::kFailed:
      throw std::runtime_error("Failed to setup attitude controller");
    case details::AttitudeControllerSetupStatus::kRejected:
      RCLCPP_WARN(logger_, "No attitude controller specified");
  }

  const auto uav_prefix = node_->declare_parameter("uav_prefix", ""s);

  setupPubSub(uav_prefix);

  initialized_ = true;
}

void AutopilotClient::setupPubSub(const std::string& uav_prefix) {
  const auto mavros_ns = uav_prefix + "/mavros";
  const auto posctl_ns = uav_prefix + "/fsc_autopilot/position_controller";
  const auto attctl_ns = uav_prefix + "/fsc_autopilot/attitude_controller";

  // Position Controller topics
  // ==========================
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      posctl_ns + "/feedback", rclcpp::QoS(1),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_timestamp_ = node_->now();
        tf2::fromMsg(msg->pose.pose.position, state_.pose.position);

        if (msg->child_frame_id == "base_link") {  // body-frame velocity
          RCLCPP_WARN_ONCE(logger_, "Using odometry feedback with body-frame velocity!");
          Eigen::Vector3d velocity_body;
          tf2::fromMsg(msg->twist.twist.linear, velocity_body);
          Eigen::Quaterniond body_to_inertial;
          tf2::fromMsg(msg->pose.pose.orientation, body_to_inertial);
          state_.twist.linear = body_to_inertial * velocity_body;
        } else {
          tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
        }
      });

  pos_ref_sub_ = node_->create_subscription<fsc_autopilot_msgs::msg::PositionControllerReference>(
      posctl_ns + "/reference", rclcpp::QoS(1),
      [this](const fsc_autopilot_msgs::msg::PositionControllerReference::SharedPtr msg) {
        fromMsg(*msg, outer_ref_);
      });

  tracking_error_pub_ = node_->create_publisher<fsc_autopilot_msgs::msg::PositionControllerState>(
      posctl_ns + "/state", 1);
  ude_state_pub_ = node_->create_publisher<fsc_autopilot_msgs::msg::UDEState>(
      posctl_ns + "/ude", 1);

  // Mavros topics
  // =============
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      mavros_ns + "/imu/data", rclcpp::QoS(1),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        const auto timestamp = rclcpp::Time(msg->header.stamp);
        // This check is fine on the first IMU message since imu_last_recv_time_
        // is initialized to 0, and ros::Time is never less than 0 by design
        if (timestamp < last_imu_timestamp_) {
          RCLCPP_WARN(logger_,
              "IMU data is older than previous IMU data (timestamps %6.3f < "
              "%6.3f)",
              timestamp.seconds(), last_imu_timestamp_.seconds());
          return;
        }

        tf2::fromMsg(msg->angular_velocity, state_.twist.angular);
        tf2::fromMsg(msg->orientation, state_.pose.orientation);
        const auto dt = (timestamp - last_imu_timestamp_).seconds();
        if (last_imu_timestamp_ == rclcpp::Time(0, 0, RCL_ROS_TIME) || dt <= 0.0) {
          tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
        } else {
          Eigen::Vector3d output;
          tf2::fromMsg(msg->linear_acceleration, output);
          state_.accel.linear = imu_filter_.update(output, dt, 40.0);
        }

        last_imu_timestamp_ = rclcpp::Time(msg->header.stamp);
      });

  vehicle_state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      mavros_ns + "/state", rclcpp::QoS(1),
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        vehicle_state_ = *msg;
      });

  setpoint_pub_ = node_->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      mavros_ns + "/setpoint_raw/attitude", 1);

  // Attitude controller topics
  // ==========================
  attitude_error_pub_ = node_->create_publisher<fsc_autopilot_msgs::msg::AttitudeControllerState>(
      attctl_ns + "/state", 1);
}

void AutopilotClient::outerLoop() {
  using tf2::toMsg;
  auto now = node_->now();
  if (!details::CheckSensorAge("odometry", now, last_odom_timestamp_, outer_period_, logger_)) {
    return;
  }
  
  // calculate time step
  static auto last_time = now;
  const auto dt = (now - last_time).seconds();
  last_time = now;
  
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // check drone status
  fsc::PositionControllerState pos_ctrl_err;
  fsc::UDEState ude_state;
  if (ude_->update(state_, input_, dt, &ude_state) != fsc::UDEErrc::kSuccess) {
    RCLCPP_ERROR(logger_, "UDE Update failed");
  }

  ude_->ude_active() =
      (vehicle_state_.connected != 0U) && (vehicle_state_.armed != 0U) &&
      (vehicle_state_.mode == "OFFBOARD" || vehicle_state_.mode == "GUIDED");

  Eigen::Vector3d ude_output;
  if (!ude_->getEstimate(ude_output)) {
    RCLCPP_WARN(logger_, "Failed to get UDE estimate");
  }
  outer_ref_.thrust = -ude_output;
  
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      pos_ctrl_->run(state_, outer_ref_, dt, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    RCLCPP_ERROR(logger_, "Position controller failed!: %s",
              outer_success.message().c_str());
    return;
  }

  input_ = fsc::VehicleInput{pos_ctrl_out};
  const auto mapped_input = mdl_.transformInputs(input_);
  const auto& [thrust, orientation_sp] = mapped_input.thrust_attitude();

  fsc_autopilot_msgs::msg::PositionControllerState tracking_error_msg;
  toMsg(tf2::Stamped(pos_ctrl_err, now, ""), tracking_error_msg);
  tracking_error_pub_->publish(tracking_error_msg);

  fsc_autopilot_msgs::msg::UDEState ude_state_msg;
  toMsg(ude_state, ude_state_msg);
  ude_state_pub_->publish(ude_state_msg);

  cmd_.thrust = static_cast<float>(thrust);
  
  // define output messages
  if (enable_inner_controller_) {
    inner_ref_.orientation = orientation_sp;
  } else {
    cmd_.header.stamp = now;
    cmd_.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                     mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                     mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    cmd_.orientation = tf2::toMsg(orientation_sp);

    setpoint_pub_->publish(cmd_);
  }
}

void AutopilotClient::innerLoop() {
  using tf2::toMsg;
  auto now = node_->now();
  if (!details::CheckSensorAge("IMU", now, last_imu_timestamp_,
                               inner_period_, logger_)) {
    return;
  }

  static auto last_time = now;
  const auto dt = (now - last_time).seconds();
  last_time = now;
  
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  fsc::AttitudeControllerState att_ctrl_err;

  const auto& [att_ctrl_out, inner_success] =
      att_ctrl_->run(state_, inner_ref_, dt, &att_ctrl_err);

  if (enable_inner_controller_) {
    if (inner_success != fsc::ControllerErrc::kSuccess) {
      RCLCPP_ERROR(logger_, "Attitude controller failed!: %s",
                inner_success.message().c_str());
      return;
    }
    cmd_.header.stamp = now;
    cmd_.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
    toMsg(att_ctrl_out.thrust_rates().body_rates, cmd_.body_rate);
    setpoint_pub_->publish(cmd_);

    fsc_autopilot_msgs::msg::AttitudeControllerState attitude_error_msg;
    toMsg(tf2::Stamped{att_ctrl_err, now, ""},
          attitude_error_msg);
    attitude_error_pub_->publish(attitude_error_msg);
  }
}