#include "tracking_control/ude/accel_based_multirotor_ude.hpp"

namespace fsc {
Eigen::Vector3d AccelBasedMultirotorUDE::computeIntegrand(
    const VehicleState& state, const VehicleInput& input,
    MultirotorUDEState* err) const {
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

Eigen::Vector3d AccelBasedMultirotorUDE::computeDamping(
    [[maybe_unused]] const VehicleState& state, MultirotorUDEState* err) const {
  if (err) {
    err->damping_term.setConstant(-1.0);
  }
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d BodyAccelBasedMultirotorUDE::computeIntegrand(
    const VehicleState& state, const VehicleInput& input,
    MultirotorUDEState* err) const {
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

Eigen::Vector3d BodyAccelBasedMultirotorUDE::computeDamping(
    [[maybe_unused]] const VehicleState& state, MultirotorUDEState* err) const {
  if (err) {
    err->damping_term.setConstant(-1.0);
  }
  return Eigen::Vector3d::Zero();
}
}  // namespace fsc
