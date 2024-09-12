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
#include "fsc_autopilot_msgs/AttitudeControllerState.h"
#include "fsc_autopilot_msgs/PositionControllerReference.h"
#include "fsc_autopilot_msgs/PositionControllerState.h"
#include "fsc_autopilot_msgs/UDEState.h"
#include "fsc_autopilot_ros/msg_conversion.hpp"
#include "fsc_autopilot_ros/ros_support.hpp"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "ros/exception.h"
#include "ros/exceptions.h"
#include "sensor_msgs/Imu.h"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

namespace details {

bool setupPositionController(std::unique_ptr<fsc::PositionControllerBase>& ctl,
                             double& rate, RosLogger& logger) {
  RosParamLoader loader{"~position_controller"};
  const auto type = loader.param("type", "robust_controller"s);
  if (type.empty()) {
    throw ros::InvalidParameterException(
        "Position controller type cannot be empty");
  }

  ctl = fsc::PositionControllerFactory::Create(type, logger);

  auto params = ctl->getParams(true);
  if (!params->load(loader, logger)) {
    ROS_FATAL("Failed to load position controller parameters");
    return false;
  }
  if (!ctl->setParams(*params, logger)) {
    ROS_FATAL("Failed to set position controller parameters");
    return false;
  }
  loader.getParam("rate", rate);
  ROS_INFO_STREAM(params->toString());
  return true;
}

std::unique_ptr<fsc::UDEBase> setupUDE(RosLogger& logger) {
  RosParamLoader loader{"~ude"};
  const auto type = loader.param("type", "velocity_based"s);

  if (type.empty()) {
    return nullptr;
  }
  auto res = fsc::UDEFactory::Create(type, logger);
  fsc::UDEParameters params;
  if (!params.load(loader, logger) || !res->setParams(params, logger)) {
    ROS_ERROR("Got invalid parameters");
    return nullptr;
  }
  ROS_INFO_STREAM(params.toString());
  return res;
}

enum class AttitudeControllerSetupStatus { kSuccess, kFailed, kRejected };

AttitudeControllerSetupStatus setupAttitudeController(
    std::unique_ptr<fsc::AttitudeControllerBase>& ctl, double& rate,
    RosLogger& logger) {
  RosParamLoader loader{"~attitude_controller"};
  const auto type = loader.param("type", "simple"s);
  if (type.empty()) {
    return AttitudeControllerSetupStatus::kRejected;
  }
  ctl = fsc::AttitudeControllerFactory::Create(type, logger);

  auto params = ctl->getParams(true);
  if (!params->load(loader, logger)) {
    ROS_FATAL("Failed to load attitude controller parameters");
    return AttitudeControllerSetupStatus::kFailed;
  }

  if (!ctl->setParams(*params, logger)) {
    ROS_FATAL("Failed to set attitude controller parameters");
    return AttitudeControllerSetupStatus::kFailed;
  }
  loader.getParam("rate", rate);

  ROS_INFO_STREAM(params->toString());
  return AttitudeControllerSetupStatus::kSuccess;
}

bool CheckSensorAge(const std::string& type, ros::Time now, ros::Time then,
                    double period) {
  constexpr int kMeasurementStaleFactor = 5;

  if (then == ros::Time{0}) {
    ROS_WARN_THROTTLE(1, "First %s measurement not yet received", type.c_str());
    return false;
  }

  const auto imu_data_age = (now - then).toSec();
  if (imu_data_age > kMeasurementStaleFactor * period) {
    ROS_WARN_THROTTLE(
        1, "Age of the last %s measurement is %fs > %dx control period %fs",
        type.c_str(), imu_data_age, kMeasurementStaleFactor, period);
  }
  return true;
}

}  // namespace details

static constexpr double kDefaultPositionControllerRate = 30.0;
static constexpr double kDefaultAttitudeControllerRate = 250.0;

AutopilotClient::AutopilotClient() {
  ros::NodeHandle pnh("~");

  if (fsc::VehicleModelParameters params;
      !params.load(RosParamLoader{"~vehicle"}, logger_) ||
      !mdl_.setParams(params, logger_)) {
    throw ros::InvalidParameterException(
        "Failed to get required vehicle model parameters");
  }

  // Call out the type of the vehicle to the user
  const std::string ruler(80, '#');
  ROS_INFO("\033[1;32m\n%s\n    Starting autopilot for vehicle: %-45s\n%s",
           ruler.c_str(), mdl_.vehicle_name.c_str(), ruler.c_str());

  auto outer_rate = kDefaultPositionControllerRate;
  if (!details::setupPositionController(pos_ctrl_, outer_rate, logger_)) {
    throw ros::Exception("Failed to setup position controller");
  }
  outer_period_ = ros::Duration{ros::Rate{outer_rate}}.toSec();
  outer_loop_ = nh_.createTimer(outer_rate, &AutopilotClient::outerLoop, this);
  ROS_INFO("Running position controller at %f hz", outer_rate);

  ude_ = details::setupUDE(logger_);

  auto inner_rate = kDefaultAttitudeControllerRate;
  switch (details::setupAttitudeController(att_ctrl_, inner_rate, logger_)) {
    case details::AttitudeControllerSetupStatus::kSuccess:
      inner_period_ = ros::Duration{ros::Rate{inner_rate}}.toSec();
      inner_loop_ =
          nh_.createTimer(inner_rate, &AutopilotClient::innerLoop, this);

      enable_inner_controller_ = pnh.param("enable_inner_controller", true);
      ROS_INFO("Running attitude controller at %f hz; Initially %sabled",
               inner_rate, enable_inner_controller_ ? "en" : "dis");

      break;
    case details::AttitudeControllerSetupStatus::kFailed:
      throw ros::Exception("Failed to setup attitude controller");
    case details::AttitudeControllerSetupStatus::kRejected:
      ROS_WARN("No attitude controller specified");
  }

  const auto uav_prefix = pnh.param("uav_prefix", ""s);

  setupPubSub(uav_prefix);

  initialized_ = true;
}

void AutopilotClient::setupPubSub(const std::string& uav_prefix) {
  const auto mavros_ns = uav_prefix + "/mavros";
  const auto posctl_ns = uav_prefix + "/fsc_autopilot/position_controller";
  const auto attctl_ns = uav_prefix + "/fsc_autopilot/attitude_controller";

  // Position Controller topics
  // ==========================
  //
  // Disambiguating odometry:
  // * - http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  //   - The pose in this message should be specified in the coordinate frame
  //   given by header.frame_id.
  //   - The twist in this message should be specified in the coordinate frame
  //   given by the child_frame_id
  //
  // * - https://docs.px4.io/v1.14/en/ros/external_position_estimation.html
  //   - "Note that if you are sending odometry data to px4 using child_frame_id
  //   = base_link, then you need to make sure that the twist portion of the
  //   nav_msgs/Odometry message is expressed in body frame, not in inertial
  //   frame!!!!!."
  subs_.emplace_back(nh_.subscribe<nav_msgs::Odometry>(
      posctl_ns + "/feedback", 1, [this](auto&& msg) {
        last_odom_timestamp_ = ros::Time::now();
        tf2::fromMsg(msg->pose.pose.position, state_.pose.position);

        if (msg->child_frame_id == "base_link") {  // body-frame velocity
          ROS_WARN_ONCE("Using odometry feedback with body-frame velocity!");
          Eigen::Vector3d velocity_body;
          tf2::fromMsg(msg->twist.twist.linear, velocity_body);
          Eigen::Quaterniond body_to_inertial;
          tf2::fromMsg(msg->pose.pose.orientation, body_to_inertial);
          state_.twist.linear = body_to_inertial * velocity_body;
        } else {
          tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
        }
      }));

  subs_.emplace_back(
      nh_.subscribe<fsc_autopilot_msgs::PositionControllerReference>(
          posctl_ns + "/reference", 1,
          [this](auto&& msg) { fromMsg(*msg, outer_ref_); }));

  tracking_error_pub_ =
      nh_.advertise<fsc_autopilot_msgs::PositionControllerState>(
          posctl_ns + "/state", 1);
  ude_state_pub_ =
      nh_.advertise<fsc_autopilot_msgs::UDEState>(posctl_ns + "/ude", 1);
  // Mavros topics
  // =============
  subs_.emplace_back(nh_.subscribe<sensor_msgs::Imu>(
      mavros_ns + "/imu/data", 1, [this](auto&& msg) {
        const auto& timestamp = msg->header.stamp;
        // This check is fine on the first IMU message since imu_last_recv_time_
        // is initialized to 0, and ros::Time is never less than 0 by design
        if (timestamp < last_imu_timestamp_) {
          ROS_WARN(
              "IMU data is older than previous IMU data (timestamps %6.3f < "
              "%6.3f)",
              timestamp.toSec(), last_imu_timestamp_.toSec());
          return;
        }

        tf2::fromMsg(msg->angular_velocity, state_.twist.angular);
        tf2::fromMsg(msg->orientation, state_.pose.orientation);
        const auto dt = (timestamp - last_imu_timestamp_).toSec();
        if (last_imu_timestamp_ == ros::Time(0) || dt <= 0.0) {
          tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
        } else {
          Eigen::Vector3d output;
          tf2::fromMsg(msg->linear_acceleration, output);
          state_.accel.linear = imu_filter_.update(output, dt, 40.0);
        }

        last_imu_timestamp_ = msg->header.stamp;
      }));

  subs_.emplace_back(nh_.subscribe<mavros_msgs::State>(
      mavros_ns + "/state", 1, [this](auto&& msg) { vehicle_state_ = *msg; }));

  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      mavros_ns + "/setpoint_raw/attitude", 1);

  // Attitude controller topics
  // ==========================
  attitude_error_pub_ =
      nh_.advertise<fsc_autopilot_msgs::AttitudeControllerState>(
          attctl_ns + "/state", 1);
}

void AutopilotClient::outerLoop(const ros::TimerEvent& event) {
  using tf2::toMsg;
  if (!details::CheckSensorAge("odometry", event.current_real,
                               last_odom_timestamp_, outer_period_)) {
    return;
  }
  // calculate time step
  const auto dt = (event.current_real - event.last_real).toSec();
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // check drone status

  fsc::PositionControllerState pos_ctrl_err;
  fsc::UDEState ude_state;
  if (ude_->update(state_, input_, dt, &ude_state) != fsc::UDEErrc::kSuccess) {
    ROS_ERROR("UDE Update failed");
  }

  ude_->ude_active() =
      (vehicle_state_.connected != 0U) && (vehicle_state_.armed != 0U) &&
      (vehicle_state_.mode == "OFFBOARD" || vehicle_state_.mode == "GUIDED");

  Eigen::Vector3d ude_output;
  if (!ude_->getEstimate(ude_output)) {
    ROS_WARN("Failed to get UDE estimate");
  }
  outer_ref_.thrust = -ude_output;
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      pos_ctrl_->run(state_, outer_ref_, dt, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    ROS_ERROR("Position controller failed!: %s",
              outer_success.message().c_str());
    return;
  }

  input_ = fsc::VehicleInput{pos_ctrl_out};
  const auto mapped_input = mdl_.transformInputs(input_);
  const auto& [thrust, orientation_sp] = mapped_input.thrust_attitude();

  fsc_autopilot_msgs::PositionControllerState tracking_error_msg;
  toMsg(tf2::Stamped(pos_ctrl_err, event.current_real, ""), tracking_error_msg);
  tracking_error_pub_.publish(tracking_error_msg);

  fsc_autopilot_msgs::UDEState ude_state_msg;
  toMsg(ude_state, ude_state_msg);
  ude_state_pub_.publish(ude_state_msg);

  cmd_.thrust = static_cast<float>(thrust);
  // define output messages
  if (enable_inner_controller_) {
    inner_ref_.orientation = orientation_sp;
  } else {
    cmd_.header.stamp = event.current_real;
    cmd_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                     mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                     mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    cmd_.orientation = tf2::toMsg(orientation_sp);

    setpoint_pub_.publish(cmd_);
  }
  // convert thrust command to normalized thrust input
  // if the thrust is an acc command, then the thrust curve must change as
  // well
}

void AutopilotClient::innerLoop(const ros::TimerEvent& event) {
  using tf2::toMsg;
  if (!details::CheckSensorAge("IMU", event.current_real, last_imu_timestamp_,
                               inner_period_)) {
    return;
  }

  const auto dt = (event.current_real - event.last_real).toSec();
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  fsc::AttitudeControllerState att_ctrl_err;

  const auto& [att_ctrl_out, inner_success] =
      att_ctrl_->run(state_, inner_ref_, dt, &att_ctrl_err);

  if (enable_inner_controller_) {
    if (inner_success != fsc::ControllerErrc::kSuccess) {
      ROS_ERROR("Attitude controller failed!: %s",
                inner_success.message().c_str());
      return;
    }
    cmd_.header.stamp = event.current_real;
    cmd_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    toMsg(att_ctrl_out.thrust_rates().body_rates, cmd_.body_rate);
    setpoint_pub_.publish(cmd_);

    fsc_autopilot_msgs::AttitudeControllerState attitude_error_msg;
    toMsg(tf2::Stamped{att_ctrl_err, event.current_real, ""},
          attitude_error_msg);
    attitude_error_pub_.publish(attitude_error_msg);
  }
}

}  // namespace nodelib
