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

#include "fsc_autopilot/attitude_control/apm_attitude_controller.hpp"

#include "Eigen/Dense"
#include "fsc_autopilot/attitude_control/attitude_control_error.hpp"
#include "fsc_autopilot/attitude_control/attitude_controller_factory.hpp"
#include "fsc_autopilot/attitude_control/control.hpp"
#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/math/rotation.hpp"

namespace fsc {

AttitudeControlResult APMAttitudeController::run(
    const VehicleState& state, const AttitudeReference& refs, double dt,
    [[maybe_unused]] ContextBase* error) {
  if (dt <= 0) {
    return {VehicleInput{ThrustRates{0.0, Eigen::Vector3d::Zero()}},
            ControllerErrc::kTimestepError};
  }

  const auto [orientation_ref, heading_angle, heading_rate_raw] = refs;
  const auto slew_yaw_max = ang_vel_max_.z();
  const auto heading_rate =
      std::clamp(heading_rate_raw, -slew_yaw_max, slew_yaw_max);

  // convert thrust vector and heading to a quaternion attitude
  const Eigen::Quaterniond desired_attitude_quat = refs.orientation;

  if (rate_bf_ff_enabled_) {
    // calculate the angle error in x and y.
    const Eigen::Vector3d attitude_error =
        apm::ThrustVectorRotationAngles(desired_attitude_quat, attitude_target_)
            .attitude_error;

    // When yaw acceleration limiting is enabled, the yaw input shaper
    // constrains angular acceleration about the yaw axis, slewing the output
    // rate towards the input rate.
    ang_vel_target_.x() =
        apm::InputShapingAngle(attitude_error.x(), input_tc_,
                               ang_accel_max_.x(), ang_vel_target_.x(), dt);
    ang_vel_target_.y() =
        apm::InputShapingAngle(attitude_error.y(), input_tc_,
                               ang_accel_max_.y(), ang_vel_target_.y(), dt);
    ang_vel_target_.z() = apm::InputShapingAngle(
        attitude_error.z(), input_tc_, ang_accel_max_.z(), ang_vel_target_.z(),
        heading_rate, ang_vel_max_.z(), dt);

    // Limit the angular velocity
    ang_vel_target_ = apm::BodyRateLimiting(ang_vel_target_, ang_vel_max_);
  } else {
    // set persisted quaternion target attitude
    attitude_target_ = desired_attitude_quat;

    // Set rate feedforward requests to zero
    ang_vel_target_.setZero();
  }

  // Call quaternion attitude controller
  auto [ang_vel_body, attitude_error] = attitudeControllerRunQuat(
      state.pose.orientation, state.twist.angular, dt);

  AttitudeControlError* err = nullptr;
  if ((error != nullptr) && error->name() == "attitude_control_error") {
    err = static_cast<decltype(err)>(error);
  }

  if (err) {
    err->attitude_error = attitude_error;
  }

  return {VehicleInput{0.0, ang_vel_body}, ControllerErrc::kSuccess};
}

auto APMAttitudeController::attitudeControllerRunQuat(
    const Eigen::Quaterniond& orientation, const Eigen::Vector3d& body_rates,
    double dt) -> SetpointAndError {
  // This represents a quaternion rotation in NED frame to the body

  // This vector represents the angular error to rotate the thrust vector
  // using x and y and heading using z
  const auto& [_, attitude_error, thrust_error_angle] =
      apm::ThrustHeadingRotationAngles(attitude_target_, orientation,
                                       kp_angle_.z(), kp_yawrate_,
                                       ang_accel_max_.z());

  // Compute the angular velocity corrections in the body frame from the
  // attitude error
  // ensure angular velocity does not go over configured limits
  Eigen::Vector3d ang_vel_fb =
      updateAngVelTargetFromAttError(attitude_error, dt);
  ang_vel_fb = apm::BodyRateLimiting(ang_vel_fb, ang_vel_max_);

  // rotation from the target frame to the body frame
  const Eigen::Quaterniond rotation_target_to_body =
      orientation.inverse() * attitude_target_;

  // target angle velocity vector in the body frame
  const Eigen::Vector3d ang_vel_ff = rotation_target_to_body * ang_vel_target_;

  constexpr double kThrustErrorAngleThresh = deg2rad(30.0);

  // Correct the thrust vector and smoothly add feedforward and yaw input
  feedforward_scalar_ = 1.0;
  Eigen::Vector3d ang_vel_body;
  if (thrust_error_angle > kThrustErrorAngleThresh * 2.0) {
    ang_vel_body << ang_vel_fb.head<2>(), body_rates.z();
  } else if (thrust_error_angle > kThrustErrorAngleThresh) {
    feedforward_scalar_ = 1.0 - (thrust_error_angle - kThrustErrorAngleThresh) /
                                    kThrustErrorAngleThresh;
    ang_vel_body << ang_vel_fb.head<2>() +
                        ang_vel_ff.head<2>() * feedforward_scalar_,
        body_rates.z() * (1.0 - feedforward_scalar_) +
            (ang_vel_fb.z() + ang_vel_ff.z()) * feedforward_scalar_;
  } else {
    ang_vel_fb = ang_vel_fb + ang_vel_ff;
  }

  if (rate_bf_ff_enabled_) {
    // rotate target and normalize
    attitude_target_ *= fsc::AngleAxisToQuaternion(ang_vel_target_ * dt);
  }

  return {ang_vel_fb, attitude_error};
}

Eigen::Vector3d APMAttitudeController::updateAngVelTargetFromAttError(
    const Eigen::Vector3d& attitude_error, double dt) const {
  // minimum body-frame acceleration limit for the stability controller (for
  // roll and pitch axis)
  constexpr double kMinRollPitchAccel = deg2rad(40.0);

  // maximum body-frame acceleration limit for the stability controller (for
  // roll and pitch axis)
  constexpr double kMaxRollPitchAccel = deg2rad(720.0);
  // minimum body-frame acceleration limit for the stability controller (for
  // yaw axis)
  constexpr double kMinYawAccel = deg2rad(10.0);

  // maximum body-frame acceleration limit for the stability controller (for
  // yaw axis)
  constexpr double kMaxYawAccel = deg2rad(120.0);
  if (!use_sqrt_controller_) {
    return kp_angle_.cwiseProduct(attitude_error);
  }
  Eigen::Vector3d body_rate_target;
  // Compute the roll angular velocity demand from the roll angle error
  if (ang_accel_max_.x() > 0.0) {
    body_rate_target.x() = apm::PiecewiseProportionalSqrt(
        attitude_error.x(), kp_angle_.x(),
        std::clamp(ang_accel_max_.x() / 2.0, kMinRollPitchAccel,
                   kMaxRollPitchAccel),
        dt);
  } else {
    body_rate_target.x() = kp_angle_.x() * attitude_error.x();
  }

  // Compute the pitch angular velocity demand from the pitch angle error
  if (ang_accel_max_.y() > 0.0) {
    body_rate_target.y() = apm::PiecewiseProportionalSqrt(
        attitude_error.y(), kp_angle_.y(),
        std::clamp(ang_accel_max_.y() / 2.0, kMinRollPitchAccel,
                   kMaxRollPitchAccel),
        dt);
  } else {
    body_rate_target.y() = kp_angle_.y() * attitude_error.y();
  }

  // Compute the yaw angular velocity demand from the yaw angle error
  if (ang_accel_max_.z() > 0.0) {
    body_rate_target.z() = apm::PiecewiseProportionalSqrt(
        attitude_error.z(), kp_angle_.z(),
        std::clamp(ang_accel_max_.z() / 2.0, kMinYawAccel, kMaxYawAccel), dt);
  } else {
    body_rate_target.z() = kp_angle_.z() * attitude_error.z();
  }

  return body_rate_target;
}

bool APMAttitudeController::setParams(const ParameterBase& params,
                                      LoggerBase* logger) {
  if (params.parameterFor() != "apm_attitude_controller") {
    LOG_OPTIONAL(logger, Severity::kError,
                 "Mismatch in parameter and receiver");
    return false;
  }

  if (!params.valid()) {
    LOG_OPTIONAL(logger, Severity::kError, "Parameters are invalid");
    return false;
  }

  const auto& p = static_cast<const APMAttitudeControllerParams&>(params);

  rate_bf_ff_enabled_ = p.rate_bf_ff_enabled;
  use_sqrt_controller_ = p.use_sqrt_controller;
  input_tc_ = p.input_tc;
  kp_yawrate_ = p.kp_yawrate;
  kp_angle_ = p.kp_angle;
  ang_accel_max_ = p.ang_accel_max;
  ang_vel_max_ = p.ang_vel_max;
  parameters_valid_ = true;
  return true;
}

bool APMAttitudeControllerParams::load(const ParameterLoaderBase& loader,
                                       LoggerBase* logger) {
  std::ignore = loader.getParam("input_tc", input_tc);
  if (input_tc < 0.0) {
    logger->log(Severity::kError, "`input_tc` must not be negative");
    return true;
  }
  std::ignore = loader.getParam("use_sqrt_controller", use_sqrt_controller);
  std::ignore = loader.getParam("enable_rate_feedforward", rate_bf_ff_enabled);
  std::ignore = loader.getParam("roll_p", kp_angle.x());
  std::ignore = loader.getParam("pitch_p", kp_angle.y());
  std::ignore = loader.getParam("yaw_p", kp_angle.z());
  std::ignore = loader.getParam("yawrate_p", kp_yawrate);
  std::ignore = loader.getParam("rollrate_max", ang_vel_max.x());
  std::ignore = loader.getParam("pitchrate_max", ang_vel_max.y());
  std::ignore = loader.getParam("yawrate_max", ang_vel_max.z());
  std::ignore = loader.getParam("roll_accel_max", ang_accel_max.x());
  std::ignore = loader.getParam("pitch_accel_max", ang_accel_max.y());
  std::ignore = loader.getParam("yaw_accel_max", ang_accel_max.z());
  return true;
}

std::string APMAttitudeControllerParams::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};

  std::ostringstream oss;

  oss << "APM Attitude Controller parameters:" << std::boolalpha     //
      << "\nrate_bf_ff_enabled: " << rate_bf_ff_enabled              //
      << "\nuse_sqrt_controller: " << use_sqrt_controller            //
      << "\ninput_tc: " << input_tc                                  //
      << "\nkp_angle: " << kp_angle.transpose().format(f)            //
      << "\nang_accel_lax: " << ang_accel_max.transpose().format(f)  //
      << "\nang_vel_lax: " << ang_vel_max.transpose().format(f);     //
  return oss.str();
}

REGISTER_ATTITUDE_CONTROLLER(APMAttitudeController, "apm");
}  // namespace fsc
