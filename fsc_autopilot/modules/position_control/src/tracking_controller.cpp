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

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/position_control/control.hpp"

namespace fsc {
TrackingController::TrackingController(ParametersSharedPtr params)
    : params_(std::move(params)) {}

TrackingController::TrackingController(ParametersSharedPtr params,
                                       UDESharedPtr ude)
    : params_(std::move(params)), ude_(std::move(ude)) {}

ControlResult TrackingController::run(const VehicleState& state,
                                      const Reference& refs,
                                      ContextBase* error) {
  using std::atan2;

  if (params_ == nullptr || !params_->valid()) {
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

  if (params_->apply_pos_err_saturation) {
    position_error = SaturationSmoothing(raw_position_error, 1.0);
  } else {
    position_error = raw_position_error;
  }

  if (params_->apply_vel_err_saturation) {
    velocity_error = SaturationSmoothing(raw_velocity_error, 1.0);
  } else {
    velocity_error = raw_velocity_error;
  }

  // SaturationSmoothing(raw_velocity_error, 1.0);

  // bound the velocity and position error
  // kv * (ev + kp * sat(ep))
  // x and y direction
  const Eigen::Vector3d feedback = params_->k_vel.cwiseProduct(
      velocity_error + params_->k_pos.cwiseProduct(position_error));

  Eigen::Vector3d ude_value = Eigen::Vector3d::Zero();
  if (ude_) {
    auto ude_result = ude_->update(
        state, VehicleInput{ThrustAttitude{scalar_thrust_setpoint_}},
        err ? &err->ude_state : nullptr);
    if (ude_result != UDEErrc::kSuccess) {
      return {getFallBackSetpoint(), ControllerErrc::kSubcomponentError};
    }

    if (!ude_->getEstimate(ude_value)) {
      return {getFallBackSetpoint(), ControllerErrc::kSubcomponentError};
    }
  } else {
    return {getFallBackSetpoint(), ControllerErrc::kSubcomponentMissing};
  }

  // m * g * [0,0,1]
  const Eigen::Vector3d vehicle_weight = -params_->vehicle_mass * kGravity;

  // virtual control force: TO DO: check this function so that it take bounds
  // as arguments
  Eigen::Vector3d thrust_setpoint_raw = -feedback - vehicle_weight - ude_value;
  Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, {params_->min_thrust, params_->max_thrust},
      params_->max_tilt_angle);

  // attitude target
  Eigen::Matrix3d attitude_sp =
      thrustVectorToRotation(thrust_setpoint, ref_yaw);

  // total required thrust
  scalar_thrust_setpoint_ =
      thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());
  // std::cout << "thrust setpoint (N) is: " << thrust_sp_ << '\n';
  //  required thrust per rotor
  double thrust_per_rotor =
      scalar_thrust_setpoint_ / static_cast<double>(params_->num_of_rotors);
  // std::cout << "thrust per rotor (N) is: " << thrust_per_rotor << '\n';

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input =
      VehicleInput{thrust_per_rotor, Eigen::Quaterniond(attitude_sp)};
  if (err) {
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->feedback = feedback;
    err->thrust_setpoint = thrust_setpoint;
    // msg output
    err->scalar_thrust_sp = scalar_thrust_setpoint_;
    err->thrust_per_rotor =
        scalar_thrust_setpoint_ / static_cast<double>(params_->num_of_rotors);
  }

  return result;
}

std::string TrackingControllerParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "Quadrotor Mass: " << vehicle_mass << "\n"
      << "thrust bounds: [" << min_thrust << "," << max_thrust << "]\n"
      << "Tracking Controller parameters:\nk_pos: "
      << k_pos.transpose().format(f)                  //
      << "\nk_vel: " << k_vel.transpose().format(f);  //

  return oss.str();
}

bool TrackingControllerParameters::load(const ParameterLoaderBase& loader,
                                        LoggerBase* logger) {
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
    if (logger) {
      logger->log(Severity::kError, "Failed to load parameter `vehicle_mass`");
    }
    return false;
  }

  std::ignore = loader.getParam("min_thrust", min_thrust);

  std::ignore = loader.getParam("max_thrust", max_thrust);

  std::ignore = loader.getParam("max_tilt_angle", max_tilt_angle);
  return true;
}
}  // namespace fsc