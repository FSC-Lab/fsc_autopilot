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
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/lqg/lqg_factory.hpp"
#include "fsc_autopilot/position_control/control.hpp"
#include "fsc_autopilot/ude/ude_factory.hpp"

namespace fsc {

// Main function to run the tracking controller

ControlResult TrackingController::run(const VehicleState& state,
                                      const Reference& refs, double dt,
                                      ContextBase* error) {
  using std::atan2;

  // Check if parameters are valid

  if (!params_valid_) {
    return {getFallBackSetpoint(), ControllerErrc::kInvalidParameters};
  }

  // Extract current state and reference values

  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;
  const auto& curr_acceleration = state.accel.linear;
  const auto& [ref_position, ref_orientation] = refs.state.pose;
  const auto& ref_velocity = refs.state.twist.linear;
  const auto& ref_yaw = refs.yaw;

  // get current euler angles from current quaternion
  float quat[4];
  quat[0] = curr_orientation.w();
  quat[1] = curr_orientation.x();
  quat[2] = curr_orientation.y();
  quat[3] = curr_orientation.z();

  Eigen::Vector3d current_euler;
  current_euler[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                           1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
  current_euler[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
  current_euler[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                           1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));

  // define current state x
  Eigen::Matrix<double, 7, 1> x;
  x << curr_position, curr_velocity, current_euler[2];

  // initialize estimated state
  if (!lqg_active_) {
    // std::cout << "x_hat initialized" << "\n\n";
    x_hat = x;
  }

  std::cout << "x_hat = " << x_hat << "\n\n";

  // Initialize error pointer
  TrackingControllerError* err = nullptr;
  if ((error != nullptr) && error->name() == "tracking_controller.error") {
    err = static_cast<decltype(err)>(error);
  }

  Eigen::Vector3d estimated_position;
  Eigen::Vector3d estimated_velocity;
  estimated_position << x_hat[0], x_hat[1], x_hat[2];
  estimated_velocity << x_hat[3], x_hat[4], x_hat[5];
  double estimated_yaw = x_hat[6];

  // Calculate raw position and velocity errors
  // const Eigen::Vector3d raw_position_error = curr_position - ref_position;
  // const Eigen::Vector3d raw_velocity_error = curr_velocity - ref_velocity;

  const Eigen::Vector3d raw_position_error = estimated_position - ref_position;
  const Eigen::Vector3d raw_velocity_error = estimated_velocity - ref_velocity;

  // rotate the error from world frame to body frame
  Eigen::Matrix<double, 3, 3> R_b_i;
  R_b_i = yawToRotation(ref_yaw);

  const Eigen::Vector3d position_error_body_frame = R_b_i * raw_position_error;
  const Eigen::Vector3d velocity_error_body_frame = R_b_i * raw_velocity_error;

  // Apply position and velocity error saturation if enabled
  Eigen::Vector3d position_error;
  Eigen::Vector3d velocity_error;

  if (apply_pos_err_saturation_) {
    position_error = SaturationSmoothing(position_error_body_frame, 1.0);
  } else {
    position_error = position_error_body_frame;
  }

  if (apply_vel_err_saturation_) {
    velocity_error = SaturationSmoothing(velocity_error_body_frame, 1.0);
  } else {
    velocity_error = velocity_error_body_frame;
  }

  // Calculate vehicle weight m * g * [0,0,1]
  const Eigen::Vector3d vehicle_weight = -vehicle_mass_ * kGravity;

  ////////////////////// LQG controller /////////////////////////
  // Step 1: Initialize the 4x7 matrix nominal control gain
  Eigen::Matrix<double, 4, 7> gain_K;
  // gain_K << 1, 0, 0, 1.1865, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0,
  // -1, 0, 0, 0, 0, 0, 0, 0, -1;
  // original setup with eye(7), z-axis reaction slow, pitch and roll too
  // aggressive

  // gain_K << 0.1, 0, 0, 0.2253, 0, 0, 0, 0, -0.1, 0, 0, -0.1, 0, 0, 0, 0, -1,
  // 0,
  //     0, -1, 0, 0, 0, 0, 0, 0, 0, -0.1;
  // decreased row 1, 2, 4, 5, 7 of eye(7) to 0.1
  // x and y much better, z-axis still reacting slow

  gain_K << 0.25, 0, 0, 0.4055, 0, 0, 0, 0, -0.25, 0, 0, -0.25, 0, 0, 0, 0, -10,
      0, 0, -10, 0, 0, 0, 0, 0, 0, 0, -1;
  // increased row 3, 6 of eye(7) to 10

  // Step 2: Initialize the 7x1 matrix combined_error-feedback -
  // vehicle_weight
  // - ude_value
  Eigen::Matrix<double, 7, 1> combined_error;
  combined_error << position_error(0, 0), position_error(1, 0),
      position_error(2, 0), velocity_error, ref_yaw;

  // Step 3: Perform the matrix multiplication to get matrix control_input_u
  Eigen::Matrix<double, 4, 1> control_input_u = -gain_K * combined_error;

  double roll_saturated = std::min(control_input_u(1, 0), 0.75);
  double pitch_saturated = std::min(control_input_u(0, 0), 0.75);

  // update KF

  const Eigen::Vector3d position_body_frame = R_b_i * curr_position;
  const Eigen::Vector3d velocity_body_frame = R_b_i * curr_velocity;

  Eigen::Matrix<double, 7, 1> x_body_frame;

  x_body_frame << position_body_frame, velocity_body_frame, ref_yaw;

  Eigen::Matrix<double, 4, 1> kf_control_input_u;
  kf_control_input_u << pitch_saturated, roll_saturated, control_input_u(3, 0),
      0;

  Eigen::Matrix<double, 7, 1> x_hat_body;

  x_hat_body = ude_->UDEBase::updateKF(x_body_frame, kf_control_input_u, dt);

  Eigen::Vector3d position_hat_body;
  Eigen::Vector3d velocity_hat_body;
  position_hat_body << x_hat_body[0], x_hat_body[1], x_hat_body[2];
  velocity_hat_body << x_hat_body[3], x_hat_body[4], x_hat_body[5];

  // rotate the error from body frame to world frame
  Eigen::Matrix<double, 3, 3> R_i_b;
  R_i_b = bodyToWorld(ref_yaw);

  const Eigen::Vector3d position_hat = R_i_b * position_hat_body;
  const Eigen::Vector3d velocity_hat = R_i_b * velocity_hat_body;

  x_hat << position_hat, velocity_hat, ref_yaw;

  // std::cout << "x_hat = " << x_hat << "\n\n";
  std::cout << "x = " << x << "\n\n";

  Eigen::Vector3d euler_angles;
  euler_angles << roll_saturated, pitch_saturated, ref_yaw;

  Eigen::Matrix3d attitude_sp;

  attitude_sp = eulerAnglesToRotationMatrix(euler_angles);  // roll pitch yaw

  // Compute raw thrust setpoint
  Eigen::Vector3d thrust_setpoint_raw =
      control_input_u(2, 0) * Eigen::Vector3d::UnitZ() - vehicle_weight;

  // Apply thrust limiting to get final thrust setpoint
  Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, thrust_bnds_, max_tilt_angle_);

  // Calculate total required thrust
  scalar_thrust_setpoint_ =
      thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());

  double thrust_per_rotor =
      scalar_thrust_setpoint_ / static_cast<double>(num_rotors_);

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input =
      VehicleInput{thrust_per_rotor, Eigen::Quaterniond(attitude_sp)};

  // If there is an error pointer, populate error fields
  if (err) {
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->thrust_setpoint = thrust_setpoint;
    // msg output
    err->scalar_thrust_sp = scalar_thrust_setpoint_;
    err->thrust_per_rotor =
        scalar_thrust_setpoint_ / static_cast<double>(num_rotors_);
  }

  std::cout << "XSY controller is working \n\n";

  return result;
}

ControlResult TrackingControllerOriginal::run(const VehicleState& state,
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

  const Eigen::Vector3d feedback =
      k_vel_.cwiseProduct(velocity_error + k_pos_.cwiseProduct(position_error));

  Eigen::Vector3d ude_value = Eigen::Vector3d::Zero();

  if (ude_) {
    auto ude_result = ude_->UDEBase::update(
        state, VehicleInput{ThrustAttitude{scalar_thrust_setpoint_}}, dt,
        err ? &err->ude_state : nullptr);
    if (ude_result != UDEErrc::kSuccess) {
      return {getFallBackSetpoint(), ControllerErrc::kSubcomponentError};
    }

    if (!ude_->UDEBase::getEstimate(ude_value)) {
      return {getFallBackSetpoint(), ControllerErrc::kSubcomponentError};
    }
  } else {
    return {getFallBackSetpoint(), ControllerErrc::kSubcomponentMissing};
  }

  // m * g * [0,0,1]
  const Eigen::Vector3d vehicle_weight = -vehicle_mass_ * kGravity;

  Eigen::Vector3d thrust_setpoint_raw = -feedback - vehicle_weight - ude_value;
  Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, thrust_bnds_, max_tilt_angle_);

  // attitude target
  Eigen::Matrix3d attitude_sp =
      thrustVectorToRotation(thrust_setpoint, ref_yaw);

  // total required thrust
  scalar_thrust_setpoint_ =
      thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());
  // std::cout << "thrust setpoint (N) is: " << thrust_sp_ << '\n';
  //  required thrust per rotor
  double thrust_per_rotor =
      scalar_thrust_setpoint_ / static_cast<double>(num_rotors_);
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
        scalar_thrust_setpoint_ / static_cast<double>(num_rotors_);
  }

  std::cout << "UDE controller is working \n\n";
  return result;
}

// Function to set controller parameters
bool TrackingController::setParams(const ParameterBase& params,
                                   LoggerBase* logger) {
  if (params.parameterFor() != "tracking_controller") {
    LOG_OPTIONAL(logger, Severity::kError,
                 "Mismatch in parameter and receiver");

    return true;
  }

  if (!params.valid()) {
    LOG_OPTIONAL(logger, Severity::kError, "Parameters are invalid");
    return false;
  }

  const auto& p = static_cast<const TrackingControllerParameters&>(params);
  num_rotors_ = p.num_of_rotors;
  apply_pos_err_saturation_ = p.apply_pos_err_saturation;
  apply_vel_err_saturation_ = p.apply_vel_err_saturation;
  vehicle_mass_ = p.vehicle_mass;
  max_tilt_angle_ = p.max_tilt_angle;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  thrust_bnds_ = {p.min_thrust, p.max_thrust};

  if (!ude_) {
    auto ude = UDEFactory::Create(p.ude_type, logger);
    if (!ude) {
      return false;
    }
    ude_ = std::move(ude);
  }
  if (!ude_->setParams(p.ude_params)) {
    LOG_OPTIONAL(logger, Severity::kError, "Failed to set UDE parameters");
    return false;
  }

  params_valid_ = true;
  return true;
}

bool TrackingControllerOriginal::setParams(const ParameterBase& params,
                                           LoggerBase* logger) {
  if (params.parameterFor() != "tracking_controller") {
    LOG_OPTIONAL(logger, Severity::kError,
                 "Mismatch in parameter and receiver");

    return true;
  }

  if (!params.valid()) {
    LOG_OPTIONAL(logger, Severity::kError, "Parameters are invalid");
    return false;
  }

  const auto& p = static_cast<const TrackingControllerParameters&>(params);
  num_rotors_ = p.num_of_rotors;
  apply_pos_err_saturation_ = p.apply_pos_err_saturation;
  apply_vel_err_saturation_ = p.apply_vel_err_saturation;
  vehicle_mass_ = p.vehicle_mass;
  max_tilt_angle_ = p.max_tilt_angle;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  thrust_bnds_ = {p.min_thrust, p.max_thrust};

  if (!ude_) {
    auto ude = UDEFactory::Create(p.ude_type, logger);
    if (!ude) {
      return false;
    }
    ude_ = std::move(ude);
  }
  if (!ude_->setParams(p.ude_params)) {
    LOG_OPTIONAL(logger, Severity::kError, "Failed to set UDE parameters");
    return false;
  }

  params_valid_ = true;
  return true;
}

// Function to toggle UDE integration

void TrackingController::toggleIntegration(bool value) {
  if (ude_) {
    ude_->lqg_active() = value;
  }
}

void TrackingControllerOriginal::toggleIntegration(bool value) {
  if (ude_) {
    ude_->ude_active() = value;
  }
}

// Function to convert tracking controller parameters to string

std::string TrackingControllerParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "Quadrotor Mass: " << vehicle_mass << "\n"
      << "thrust bounds: [" << min_thrust << "," << max_thrust << "]\n"
      << "Tracking Controller parameters:\nk_pos: "
      << k_pos.transpose().format(f)  //
      << "\nk_vel: " << k_vel.transpose().format(f) << "\n"
      << ude_params.toString();

  return oss.str();
}

// Function to load tracking controller parameters

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

  auto sub_loader = loader.getChildLoader("de");
  if (!sub_loader) {
    return false;
  }

  sub_loader->getParam("type", ude_type);
  return ude_params.load(*sub_loader, logger);
}

}  // namespace fsc
