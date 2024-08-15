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
#include "fsc_autopilot/core/vehicle_model.hpp"
#include "fsc_autopilot/math/math_extras.hpp"
#include "fsc_autopilot/position_control/tracking_controller.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot/ude/ude_factory.hpp"
#include "fsc_autopilot_msgs/AttitudeControllerState.h"
#include "fsc_autopilot_msgs/TrackingError.h"
#include "fsc_autopilot_ros/TrackingControlConfig.h"
#include "fsc_autopilot_ros/msg_conversion.hpp"
#include "fsc_autopilot_ros/ros_support.hpp"
#include "mavros_msgs/AttitudeTarget.h"
#include "ros/exceptions.h"
#include "tf2_eigen/tf2_eigen.h"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

AutopilotClient::AutopilotClient() {
  ros::NodeHandle pnh("~");

  if (fsc::VehicleModelParameters params;
      !params.load(RosParamLoader{"~vehicle"}, logger_) ||
      !mdl_.setParams(params, logger_)) {
    throw ros::InvalidParameterException(
        "Failed to get required vehicle model parameters");
  }

  auto attitude_controller_type =
      pnh.param("attitude_controller/type", "simple"s);
  att_ctrl_ =
      fsc::AttitudeControllerFactory::Create(attitude_controller_type, logger_);

  RosParamLoader ude_params_loader{"~ude"};
  const auto ude_type = ude_params_loader.param("type", "velocity_based"s);
  if (!ude_type.empty()) {
    ude_ = fsc::UDEFactory::Create(ude_type, logger_);
    if (fsc::UDEParameters params; !params.load(ude_params_loader, logger_) ||
                                   !ude_->setParams(params, logger_)) {
      throw ros::InvalidParameterException("Got invalid parameters");
    }
  }

  if (!loadParams()) {
    throw ros::InvalidParameterException("Got invalid parameters");
  }
  const ros::Rate outer_rate = pnh.param("position_controller/rate", 30);
  const ros::Rate inner_rate = pnh.param("attitude_controller/rate", 250);

  const auto mavros_ns = pnh.param("mavros_ns", "/mavros"s);

  subs_.emplace("odom"s,
                nh_.subscribe("/state_estimator/local_position/odom/UAV0", 1,
                              &AutopilotClient::odomCb, this));

  subs_.emplace("accel"s, nh_.subscribe("/mavros/imu/data", 1,
                                        &AutopilotClient::imuCb, this));

  subs_.emplace("target"s, nh_.subscribe("position_controller/target", 1,
                                         &AutopilotClient::setpointCb, this));

  subs_.emplace("state"s, nh_.subscribe("/mavros/state", 1,
                                        &AutopilotClient::mavrosStateCb, this));

  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);

  attitude_error_pub_ =
      nh_.advertise<fsc_autopilot_msgs::AttitudeControllerState>(
          "attitude_controller/output_data", 1);

  tracking_error_pub_ = nh_.advertise<fsc_autopilot_msgs::TrackingError>(
      "position_controller/output_data", 1);

  cfg_srv_.setCallback([this](auto&& PH1, auto&& PH2) {
    dynamicReconfigureCb(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2));
  });

  outer_loop_ = nh_.createTimer(outer_rate, &AutopilotClient::outerLoop, this);
  inner_loop_ = nh_.createTimer(inner_rate, &AutopilotClient::innerLoop, this);

  watchdog_ = nh_.createTimer(outer_rate, &AutopilotClient::watchdog, this);

  initialized_ = true;
}

void AutopilotClient::odomCb(const nav_msgs::OdometryConstPtr& msg) {
  // the estimation only provides position measurement
  odom_last_recv_time_ = msg->header.stamp;
  tf2::fromMsg(msg->pose.pose.position, state_.pose.position);
  tf2::fromMsg(msg->twist.twist.linear, state_.twist.linear);
}

void AutopilotClient::imuCb(const sensor_msgs::ImuConstPtr& msg) {
  imu_last_recv_time_ = msg->header.stamp;
  tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
  tf2::fromMsg(msg->angular_velocity, state_.twist.angular);
  tf2::fromMsg(msg->orientation, state_.pose.orientation);
}

void AutopilotClient::setpointCb(
    const fsc_autopilot_msgs::TrackingReferenceConstPtr& msg) {
  tf2::fromMsg(msg->pose.position, outer_ref_.state.pose.position);
  tf2::fromMsg(msg->pose.orientation, outer_ref_.state.pose.orientation);
  tf2::fromMsg(msg->twist.linear, outer_ref_.state.twist.linear);
  tf2::fromMsg(msg->twist.angular, outer_ref_.state.twist.angular);
  tf2::fromMsg(msg->accel.linear, outer_ref_.state.accel.linear);
  tf2::fromMsg(msg->accel.angular, outer_ref_.state.accel.angular);
  outer_ref_.yaw = fsc::deg2rad(msg->yaw);
}

void AutopilotClient::mavrosStateCb(const mavros_msgs::State& msg) {
  state_last_recv_time_ = msg.header.stamp;
  vehicle_state_ = msg;
}

void AutopilotClient::outerLoop(const ros::TimerEvent& event) {
  using tf2::toMsg;
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
  outer_ref_.thrust = -ude_output;
  // outerloop control
  const auto& [pos_ctrl_out, outer_success] =
      tracking_ctrl_.run(state_, outer_ref_, dt, &pos_ctrl_err);

  if (outer_success != fsc::ControllerErrc::kSuccess) {
    ROS_ERROR("Outer controller failed!: %s", outer_success.message().c_str());
    return;
  }

  input_ = pos_ctrl_out.input;
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

void AutopilotClient::watchdog(const ros::TimerEvent& /*event*/) {
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

bool AutopilotClient::loadParams() {
  // Position controller parameters
  auto tc_params = tracking_ctrl_.getParams(true);
  if (!tc_params->load(RosParamLoader{"~position_controller"}, logger_)) {
    ROS_FATAL("Failed to load tracking controller parameters");
    return false;
  }

  if (!tracking_ctrl_.setParams(*tc_params, logger_)) {
    ROS_FATAL("Failed to set tracking controller parameters");
    return false;
  }

  // Attitude controller parameters
  auto ac_params = att_ctrl_->getParams(true);
  if (!ac_params->load(RosParamLoader{"~attitude_controller"}, logger_)) {
    ROS_FATAL("Failed to load attitude controller parameters");
  }

  if (!att_ctrl_->setParams(*ac_params, logger_)) {
    ROS_FATAL("Failed to set attitude controller parameters");
    return false;
  }

  // Core client lib parameters
  ros::NodeHandle pnh("~");
  // put load parameters into a function
  enable_inner_controller_ = pnh.param("enable_inner_controller", false);

  pnh.getParam("check_reconfiguration", check_reconfiguration_);

  // offboard control mode
  if (enable_inner_controller_) {
    ROS_INFO("Inner controller (rate mode) enabled!");
    ROS_INFO_STREAM(ac_params->toString());
  } else {
    ROS_INFO("Inner controller bypassed! (attitude setpoint mode)");
  }

  ROS_INFO_STREAM(tc_params->toString());
  return true;
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

  auto ac_params_raw = att_ctrl_->getParams(false);
  if (ac_params_raw->parameterFor() == "apm_attitude_controller") {
    auto ac_params = std::static_pointer_cast<fsc::APMAttitudeControllerParams>(
        ac_params_raw);
    const auto use_sqrt_controller_prev = std::exchange(
        ac_params->use_sqrt_controller, attitude_tracking.use_sqrt_controller);
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
    if (check_reconfiguration_ && max_kp_angle_step > kMaxParamStep) {
      constexpr char const* kAxname[] = {"roll", "pitch", "yaw"};
      ROS_ERROR("Refusing reconfiguration: delta Kp[%s] = %f > %f",
                kAxname[max_idx], max_kp_angle_step, kMaxParamStep);
    }

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

  auto tc_params_raw = tracking_ctrl_.getParams(false);
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
    const auto max_k_pos_step =
        (k_pos_new - tc_params->k_pos).cwiseAbs().maxCoeff(&max_idx);
    if (check_reconfiguration_ && max_k_pos_step > kMaxParamStep) {
      ROS_ERROR("Refusing reconfiguration: ΔKp[%d] = %f > %f", max_idx,
                max_k_pos_step, kMaxParamStep);
      return;
    }

    const Eigen::Vector3d k_pos_prev =
        std::exchange(tc_params->k_pos, k_pos_new);

    const auto& velocity_tracking = tracker.velocity_tracking;
    const auto apply_vel_err_saturation_prev =
        std::exchange(tc_params->apply_vel_err_saturation,
                      velocity_tracking.apply_vel_err_saturation);

    const Eigen::Vector3d k_vel_new{velocity_tracking.vel_p_x,  //
                                    velocity_tracking.vel_p_y,  //
                                    velocity_tracking.vel_p_z};
    const auto max_k_vel_step =
        (k_vel_new - tc_params->k_vel).cwiseAbs().maxCoeff(&max_idx);
    if (check_reconfiguration_ && max_k_vel_step > kMaxParamStep) {
      ROS_ERROR("Refusing reconfiguration: ΔKv[%d] = %f > %f", max_idx,
                max_k_vel_step, kMaxParamStep);
      return;
    }
    const Eigen::Vector3d k_vel_prev =
        std::exchange(tc_params->k_vel, k_vel_new);
    tracking_ctrl_.setParams(*tc_params, logger_);

    auto ude_params = ude_->getParams(false);

    auto ude_height_threshold_prev =
        std::exchange(ude_params.ude_height_threshold, ude.height_threshold);

    const auto ude_gain_step = std::abs(ude.gain - ude_params.ude_gain);
    if (check_reconfiguration_ && ude_gain_step > kMaxParamStep) {
      ROS_ERROR("Refusing reconfiguration: Δ ude_gain = %f > %f", ude_gain_step,
                kMaxParamStep);
      return;
    }
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
