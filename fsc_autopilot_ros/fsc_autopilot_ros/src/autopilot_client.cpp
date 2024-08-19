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

#include <memory>
#include <utility>

#include "fsc_autopilot/attitude_control/apm_attitude_controller.hpp"
#include "fsc_autopilot/attitude_control/attitude_controller_factory.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/core/vehicle_model.hpp"
#include "fsc_autopilot/math/math_extras.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"
#include "fsc_autopilot/position_control/position_controller_factory.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot/ude/ude_factory.hpp"
#include "fsc_autopilot_msgs/AttitudeControllerState.h"
#include "fsc_autopilot_msgs/TrackingError.h"
#include "fsc_autopilot_msgs/TrackingReference.h"
#include "fsc_autopilot_ros/TrackingControlConfig.h"
#include "fsc_autopilot_ros/msg_conversion.hpp"
#include "fsc_autopilot_ros/ros_support.hpp"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "ros/exception.h"
#include "ros/exceptions.h"
#include "ros/master.h"
#include "tf2_eigen/tf2_eigen.h"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

namespace details {

bool setupPositionController(std::unique_ptr<fsc::PositionControllerBase>& ctl,
                             double& rate, RosLogger& logger) {
  RosParamLoader loader{"~position_controller"};
  const auto type = loader.param("type", "tracking_controller"s);
  if (type.empty()) {
    throw ros::InvalidParameterException(
        "Position controller type cannot be empty");
  }

  ctl = fsc::PositionControllerFactory::Create(type, logger);

  auto params = ctl->getParams(true);
  if (!params->load(loader, logger)) {
    ROS_FATAL("Failed to load tracking controller parameters");
    return false;
  }
  if (!ctl->setParams(*params, logger)) {
    ROS_FATAL("Failed to set tracking controller parameters");
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
      ROS_INFO("Running position controller at %f hz; Initially %sabled",
               inner_rate, enable_inner_controller_ ? "en" : "dis");

      break;
    case details::AttitudeControllerSetupStatus::kFailed:
      throw ros::Exception("Failed to setup attitude controller");
    case details::AttitudeControllerSetupStatus::kRejected:
      ROS_WARN("No attitude controller specified");
  }

  const auto uav_prefix = pnh.param("uav_prefix", ""s);

  setupPubSub(uav_prefix);

  cfg_srv_.setCallback([this](auto&& PH1, auto&& PH2) {
    dynamicReconfigureCb(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2));
  });

  initialized_ = true;
}

void AutopilotClient::setupPubSub(const std::string& uav_prefix) {
  subs_.emplace(
      "odometry"s,
      nh_.subscribe<nav_msgs::Odometry>(
          uav_prefix + "/state_estimator/local_position/odom", 1,
          [this](auto&& msg) {
            odom_last_recv_time_ = ros::Time::now();
            tf2::fromMsg(msg->pose.pose.position, state_.pose.position);
            tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
          }));

  subs_.emplace("accel"s, nh_.subscribe(uav_prefix + "/mavros/imu/data", 1,
                                        &AutopilotClient::imuCb, this));

  subs_.emplace(
      "target"s,
      nh_.subscribe<fsc_autopilot_msgs::TrackingReference>(
          uav_prefix + "/position_controller/target", 1, [this](auto&& msg) {
            tf2::fromMsg(msg->pose.position, outer_ref_.position);
            tf2::fromMsg(msg->twist.linear, outer_ref_.velocity);
            outer_ref_.yaw = fsc::deg2rad(msg->yaw);
          }));

  subs_.emplace("state"s, nh_.subscribe<mavros_msgs::State>(
                              uav_prefix + "/mavros/state", 1,
                              [this](auto&& msg) { vehicle_state_ = *msg; }));

  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      uav_prefix + "/mavros/setpoint_raw/attitude", 1);

  attitude_error_pub_ =
      nh_.advertise<fsc_autopilot_msgs::AttitudeControllerState>(
          uav_prefix + "/attitude_controller/output_data", 1);

  tracking_error_pub_ = nh_.advertise<fsc_autopilot_msgs::TrackingError>(
      uav_prefix + "/position_controller/output_data", 1);
}

void AutopilotClient::imuCb(const sensor_msgs::ImuConstPtr& msg) {
  imu_last_recv_time_ = ros::Time::now();
  auto dt = msg->header.stamp - imu_last_recv_time_;
  auto dt_s = dt.toSec();
  if (dt_s > 0.0) {
    Eigen::Vector3d output;
    tf2::fromMsg(msg->linear_acceleration, output);
    state_.accel.linear = imu_filter_.update(output, dt_s, 40.0);
  } else {
    tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
  }
  tf2::fromMsg(msg->angular_velocity, state_.twist.angular);
  tf2::fromMsg(msg->orientation, state_.pose.orientation);
}

void AutopilotClient::outerLoop(const ros::TimerEvent& event) {
  using tf2::toMsg;
  if (!details::CheckSensorAge("odometry", event.current_real,
                               odom_last_recv_time_, outer_period_)) {
    return;
  }
  // calculate time step
  const auto dt = (event.current_real - event.last_real).toSec();
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // check drone status

  fsc::TrackingControllerError pos_ctrl_err;
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
  outer_ref_.feedforward = {
      fsc::PositionControlReference::FeedForward::Type::kThrust, -ude_output};
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      pos_ctrl_->run(state_, outer_ref_, dt, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    ROS_ERROR("Outer controller failed!: %s", outer_success.message().c_str());
    return;
  }

  input_ = fsc::VehicleInput{pos_ctrl_out};
  const auto mapped_input = mdl_.transformInputs(input_);
  const auto& [thrust, orientation_sp] = mapped_input.thrust_attitude();

  fsc_autopilot_msgs::TrackingError tracking_error_msg;
  toMsg(tf2::Stamped(pos_ctrl_err, event.current_real, ""), tracking_error_msg);
  toMsg(ude_state, tracking_error_msg.ude_state);
  tracking_error_pub_.publish(tracking_error_msg);
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
  // if the thrust is an acc command, then the thrust curve must change as well
}

void AutopilotClient::innerLoop(const ros::TimerEvent& event) {
  using tf2::toMsg;
  if (!details::CheckSensorAge("IMU", event.current_real, imu_last_recv_time_,
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

void AutopilotClient::dynamicReconfigureCb(
    const fsc_autopilot_ros::TrackingControlConfig& config,
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

  std::stringstream msg;
  msg << std::boolalpha
      << "Dynamical Reconfigure Results:\nEnabled inner controller"
      << enable_inner_controller_prev << " -> " << enable_inner_controller_
      << "\n";

  if (att_ctrl_) {
    auto ac_params_raw = att_ctrl_->getParams(false);
    if (ac_params_raw->parameterFor() == "apm_attitude_controller") {
      auto ac_params =
          std::static_pointer_cast<fsc::APMAttitudeControllerParams>(
              ac_params_raw);
      const auto use_sqrt_controller_prev =
          std::exchange(ac_params->use_sqrt_controller,
                        attitude_tracking.use_sqrt_controller);
      const auto enable_rate_feedforward_prev =
          std::exchange(ac_params->rate_bf_ff_enabled,
                        attitude_tracking.enable_rate_feedforward);

      const auto input_tc_prev =
          std::exchange(ac_params->input_tc, attitude_tracking.input_tc);
      const Eigen::Vector3d kp_angle_new{attitude_tracking.roll_p,
                                         attitude_tracking.pitch_p,
                                         attitude_tracking.yaw_p};

      const auto max_kp_angle_step =
          (kp_angle_new - ac_params->kp_angle).cwiseAbs().maxCoeff(&max_idx);
      const Eigen::Vector3d kp_angle_prev =
          std::exchange(ac_params->kp_angle, kp_angle_new);
      att_ctrl_->setParams(*ac_params, logger_);

      // Display diff message after configuring attitude controller
      msg << "Using sqrt controller: " << use_sqrt_controller_prev << " -> "
          << ac_params->use_sqrt_controller << "\nenable_rate_feedforward"
          << enable_rate_feedforward_prev << " -> "
          << ac_params->rate_bf_ff_enabled << "\ninput_tc: " << input_tc_prev
          << " -> " << ac_params->input_tc
          << "\nAttitude Kp: " << kp_angle_prev.transpose().format(matlab_fmt)
          << " -> " << kp_angle_new.transpose().format(matlab_fmt) << "\n";
    }
  }

  auto tc_params_raw = pos_ctrl_->getParams(false);
  if (tc_params_raw->parameterFor() == "tracking_controller") {
    auto tc_params =
        std::static_pointer_cast<fsc::TrackingControllerParameters>(
            tc_params_raw);
    const auto& position_tracking = tracker.position_tracking;
    const auto apply_pos_err_saturation_prev =
        std::exchange(tc_params->apply_pos_err_saturation,
                      position_tracking.apply_pos_err_saturation);

    const Eigen::Vector3d k_pos_new{position_tracking.pos_p_x,  //
                                    position_tracking.pos_p_y,  //
                                    position_tracking.pos_p_z};

    const Eigen::Vector3d k_pos_prev =
        std::exchange(tc_params->k_pos, k_pos_new);

    const auto& velocity_tracking = tracker.velocity_tracking;
    const auto apply_vel_err_saturation_prev =
        std::exchange(tc_params->apply_vel_err_saturation,
                      velocity_tracking.apply_vel_err_saturation);

    const Eigen::Vector3d k_vel_new{velocity_tracking.vel_p_x,  //
                                    velocity_tracking.vel_p_y,  //
                                    velocity_tracking.vel_p_z};

    const Eigen::Vector3d k_vel_prev =
        std::exchange(tc_params->k_vel, k_vel_new);
    pos_ctrl_->setParams(*tc_params, logger_);

    auto ude_params = ude_->getParams(false);

    auto ude_height_threshold_prev =
        std::exchange(ude_params.ude_height_threshold, ude.height_threshold);

    auto ude_gain_prev = std::exchange(ude_params.ude_gain, ude.gain);
    ude_->setParams(ude_params, logger_);

    // Display diff message after configuring position controller
    msg << "Position Error Saturation: " << apply_pos_err_saturation_prev
        << " -> " << tc_params->apply_pos_err_saturation
        << "\nKp: " << k_pos_prev.transpose().format(matlab_fmt) << " -> "
        << tc_params->k_pos.transpose().format(matlab_fmt)
        << "\nVelocity Error Saturation: " << apply_vel_err_saturation_prev
        << " -> " << tc_params->apply_vel_err_saturation
        << "\nKv: " << k_vel_prev.transpose().format(matlab_fmt) << " -> "
        << tc_params->k_vel.transpose().format(matlab_fmt)
        << "\nUDE "
           "height threshold: "
        << ude_height_threshold_prev << " -> "
        << ude_params.ude_height_threshold << "\nUDE gain: " << ude_gain_prev
        << " -> " << ude_params.ude_gain;
  }
  ROS_INFO_STREAM_THROTTLE(1.0, msg.str());
}
}  // namespace nodelib
