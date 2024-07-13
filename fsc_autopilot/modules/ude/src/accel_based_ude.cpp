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

#include "fsc_autopilot/ude/accel_based_ude.hpp"

#include "fsc_autopilot/ude/ude_factory.hpp"

namespace fsc {
Eigen::Vector3d AccelBasedUDE::computeIntegrand(const VehicleState& state,
                                                const VehicleInput& input,
                                                UDEState* err) const {
  // The expected inertial acceleration: (R_ib * a_b - g) * m
  const Eigen::Vector3d dynamical_term =
      (state.pose.orientation * state.accel.linear - kGravity) *
      params_->vehicle_mass;

  // The expected thrust/acc
  // f = Rib * [0, 0, 1] * thrust
  const double scalar_thrust = input.thrust_attitude().thrust;
  const Eigen::Vector3d expected_thrust =
      state.pose.orientation * Eigen::Vector3d::UnitZ() * scalar_thrust;

  const Eigen::Vector3d vehicle_weight = params_->vehicle_mass * kGravity;

  if (err) {
    err->actuation_term = expected_thrust;
    err->dynamical_term = dynamical_term;
  }

  return dynamical_term - expected_thrust + vehicle_weight - ude_value_;
}

Eigen::Vector3d AccelBasedUDE::computeDamping(
    [[maybe_unused]] const VehicleState& state, UDEState* err) const {
  if (err) {
    err->damping_term.setConstant(-1.0);
  }
  return Eigen::Vector3d::Zero();
}

REGISTER_UDE(AccelBasedUDE, "accel_based");

Eigen::Vector3d BodyAccelBasedUDE::computeIntegrand(const VehicleState& state,
                                                    const VehicleInput& input,
                                                    UDEState* err) const {
  const auto& [velocity, body_rate] = state.twist;
  // gravity in body frame
  const Eigen::Vector3d body_gravity =
      state.pose.orientation.inverse() * kGravity;
  // dot v_b + omega x vb
  const Eigen::Vector3d body_accel = (state.accel.linear - body_gravity);
  const Eigen::Vector3d dynamical_term =
      (body_accel + body_rate.cross(velocity)) * params_->vehicle_mass;

  // The expected thrust/acc
  // f = [0, 0, 1] * thrust
  const double scalar_thrust = input.thrust_attitude().thrust;
  const Eigen::Vector3d expected_thrust =
      Eigen::Vector3d::UnitZ() * scalar_thrust;

  const Eigen::Vector3d vehicle_weight = params_->vehicle_mass * body_gravity;

  if (err) {
    err->actuation_term = expected_thrust;
    err->dynamical_term = dynamical_term;
  }

  return dynamical_term - expected_thrust + vehicle_weight - ude_value_;
}

Eigen::Vector3d BodyAccelBasedUDE::computeDamping(
    [[maybe_unused]] const VehicleState& state, UDEState* err) const {
  if (err) {
    err->damping_term.setConstant(-1.0);
  }
  return Eigen::Vector3d::Zero();
}

REGISTER_UDE(BodyAccelBasedUDE, "body_accel_based");
}  // namespace fsc
