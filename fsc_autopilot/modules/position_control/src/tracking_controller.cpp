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

#include "fsc_autopilot/position_control/tracking_controller.hpp"

#include <iostream>

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/position_control/control.hpp"

namespace fsc {

ControlResult TrackingController::run(const VehicleState& state,
                                      const Reference& refs, double dt,
                                      ContextBase* error) {
  using std::atan2;

  if (!params_valid_) {
    return {getFallBackSetpoint(), ControllerErrc::kInvalidParameters};
  }

  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;
  const auto& curr_acceleration = state.accel.linear;
  const auto& [ref_position, ref_orientation] = refs.state.pose;
  const auto& ref_velocity = refs.state.twist.linear;
  const auto& ref_yaw = refs.yaw;

  TrackingControllerError* err = nullptr;
  if ((error != nullptr) && error->name() == "tracking_controller.error") {
    err = static_cast<decltype(err)>(error);
  }
  const Eigen::Vector3d raw_position_error = curr_position - ref_position;
  const Eigen::Vector3d raw_velocity_error = curr_velocity - ref_velocity;

  Eigen::Vector3d position_error;
  Eigen::Vector3d velocity_error;

  if (apply_pos_err_saturation_) {
    position_error = SaturationSmoothing(raw_position_error, 1.0);
  } else {
    position_error = raw_position_error;
  }

  if (apply_vel_err_saturation_) {
    velocity_error = SaturationSmoothing(raw_velocity_error, 1.0);
  } else {
    velocity_error = raw_velocity_error;
  }

  // SaturationSmoothing(raw_velocity_error, 1.0);

  // bound the velocity and position error
  // kv * (ev + kp * sat(ep))
  // x and y direction
  const Eigen::Vector3d feedback =
      k_vel_.cwiseProduct(velocity_error + k_pos_.cwiseProduct(position_error));

  // m * g * [0,0,1]
  const Eigen::Vector3d vehicle_weight = -vehicle_mass_ * kGravity;

  Eigen::Vector3d thrust_setpoint_raw =
      -feedback - vehicle_weight + refs.thrust;
  Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, thrust_bnds_, max_tilt_angle_);

  // attitude target
  Eigen::Matrix3d attitude_sp =
      thrustVectorToRotation(thrust_setpoint, ref_yaw);

  double thrust = thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input = VehicleInput{thrust, Eigen::Quaterniond(attitude_sp)};
  if (err) {
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->feedback = feedback;
    err->thrust_setpoint = thrust_setpoint;
    // msg output
    err->scalar_thrust_sp = thrust;
  }

  return result;
}

bool TrackingController::setParams(const ParameterBase& params,
                                   LoggerBase& logger) {
  if (params.parameterFor() != "tracking_controller") {
    logger.log(Severity::kError, "Mismatch in parameter and receiver");

    return true;
  }

  if (!params.valid(logger)) {
    return false;
  }

  const auto& p = static_cast<const TrackingControllerParameters&>(params);
  apply_pos_err_saturation_ = p.apply_pos_err_saturation;
  apply_vel_err_saturation_ = p.apply_vel_err_saturation;
  vehicle_mass_ = p.vehicle_mass;
  max_tilt_angle_ = p.max_tilt_angle;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  thrust_bnds_ = {p.min_thrust, p.max_thrust};

  params_valid_ = true;
  return true;
}

std::shared_ptr<ParameterBase> TrackingController::getParams(
    bool use_default) const {
  if (use_default) {
    return std::make_shared<TrackingControllerParameters>();
  }
  TrackingControllerParameters params;

  params.apply_pos_err_saturation = apply_pos_err_saturation_;
  params.k_pos = k_pos_;
  params.apply_vel_err_saturation = apply_vel_err_saturation_;
  params.k_vel = k_vel_;
  params.min_thrust = thrust_bnds_.lower;
  params.max_thrust = thrust_bnds_.upper;
  params.max_tilt_angle = max_tilt_angle_;
  params.vehicle_mass = vehicle_mass_;

  return std::make_shared<TrackingControllerParameters>(std::move(params));
}

void TrackingController::toggleIntegration(bool value) {}

std::string TrackingControllerParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "Quadrotor Mass: " << vehicle_mass << "\n"
      << "thrust bounds: [" << min_thrust << "," << max_thrust << "]\n"
      << "Tracking Controller parameters:\nk_pos: "
      << k_pos.transpose().format(f)  //
      << "\nk_vel: " << k_vel.transpose().format(f);

  return oss.str();
}
bool TrackingControllerParameters::valid(LoggerBase& logger) const {
  if (min_thrust >= max_thrust) {
    logger.log(Severity::kError,
               "`min_thrust` must be strictly less than `max_thrust`");
    return false;
  }

  if (vehicle_mass < 0.0) {
    logger.log(Severity::kError, "`vehicle_mass` must be positive");
    return false;
  }
  return true;
}

bool TrackingControllerParameters::load(const ParameterLoaderBase& loader,
                                        LoggerBase& logger) {
  std::ignore =
      loader.getParam("apply_pos_err_saturation", apply_pos_err_saturation);
  k_pos <<  // Force a line break
      loader.param("k_pos/x", kDefaultKpXY),
      loader.param("k_pos/y", kDefaultKpXY),
      loader.param("k_pos/z", kDefaultKpZ);

  std::ignore =
      loader.getParam("apply_vel_err_saturation", apply_vel_err_saturation);
  k_vel <<  // Force a line break
      loader.param("k_vel/x", kDefaultKvXY),
      loader.param("k_vel/y", kDefaultKvXY),
      loader.param("k_vel/z", kDefaultKvZ);

  if (!loader.getParam("vehicle_mass", vehicle_mass)) {
    logger.log(Severity::kError, "Failed to load parameter `vehicle_mass`");
    return false;
  }

  std::ignore = loader.getParam("min_thrust", min_thrust);

  std::ignore = loader.getParam("max_thrust", max_thrust);

  std::ignore = loader.getParam("max_tilt_angle", max_tilt_angle);

  return true;
}

}  // namespace fsc
