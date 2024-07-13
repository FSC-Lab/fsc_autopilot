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

#include "fsc_autopilot/ude/velocity_based_ude.hpp"

#include "fsc_autopilot/ude/ude_factory.hpp"

namespace fsc {

Eigen::Vector3d VelocityBasedUDE::computeIntegrand(const VehicleState& state,
                                                   const VehicleInput& input,
                                                   UDEState* err) const {
  // The expected thrust/acc
  // f = Rib * [0, 0, 1] * thrust
  const double scalar_thrust = input.thrust_attitude().thrust;
  const Eigen::Vector3d expected_thrust =
      state.pose.orientation * Eigen::Vector3d::UnitZ() * scalar_thrust;

  const Eigen::Vector3d vehicle_weight = vehicle_mass_ * kGravity;

  if (err) {
    err->actuation_term = expected_thrust;
  }

  return -expected_thrust + vehicle_weight - ude_value_;
}

REGISTER_UDE(VelocityBasedUDE, "velocity_based");

Eigen::Vector3d VelocityBasedUDE::computeDamping(const VehicleState& state,
                                                 UDEState* err) const {
  Eigen::Vector3d ude_damping = ude_gain_ * vehicle_mass_ * state.twist.linear;
  if (err) {
    err->damping_term = ude_damping;
    err->dynamical_term.setConstant(-1.0);
  }
  return ude_damping;
}

Eigen::Vector3d BodyVelocityBasedUDE::computeIntegrand(
    const VehicleState& state, const VehicleInput& input, UDEState* err) const {
  const auto& [velocity, body_rate] = state.twist;
  // omega x vb
  const Eigen::Vector3d dynamical_term =
      body_rate.cross(velocity) * vehicle_mass_;

  // The expected thrust/acc
  // f = [0, 0, 1] * thrust
  const double scalar_thrust = input.thrust_attitude().thrust;
  const Eigen::Vector3d expected_thrust =
      Eigen::Vector3d::UnitZ() * scalar_thrust;

  // gravity in body frame
  const Eigen::Vector3d body_gravity =
      state.pose.orientation.inverse() * kGravity;
  const Eigen::Vector3d vehicle_weight = vehicle_mass_ * body_gravity;

  if (err) {
    err->actuation_term = expected_thrust;
  }

  return dynamical_term - expected_thrust + vehicle_weight - ude_value_;
}

Eigen::Vector3d BodyVelocityBasedUDE::computeDamping(const VehicleState& state,
                                                     UDEState* err) const {
  const Eigen::Vector3d velocity_body =
      state.pose.orientation.inverse() * state.twist.linear;
  const Eigen::Vector3d ude_damping = ude_gain_ * vehicle_mass_ * velocity_body;
  if (err) {
    err->damping_term = ude_damping;
    err->dynamical_term.setConstant(-1.0);
  }
  return ude_damping;
}

REGISTER_UDE(BodyVelocityBasedUDE, "body_velocity_based");
}  // namespace fsc
