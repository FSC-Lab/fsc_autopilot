#include "tracking_control/ude/velocity_based_multirotor_ude.hpp"

namespace fsc {

Eigen::Vector3d VelocityBasedMultirotorUDE::computeIntegrand(
    const VehicleState& state, const VehicleInput& input,
    MultirotorUDEState* err) const {
  // The expected thrust/acc
  // f = Rib * [0, 0, 1] * thrust
  const double scalar_thrust = input.thrust_attitude().thrust;
  const Eigen::Vector3d expected_thrust =
      state.pose.orientation * Eigen::Vector3d::UnitZ() * scalar_thrust;

  const Eigen::Vector3d vehicle_weight = params_->vehicle_mass * kGravity;

  if (err) {
    err->actuation_term = expected_thrust;
  }

  return -expected_thrust + vehicle_weight - ude_value_;
}

Eigen::Vector3d VelocityBasedMultirotorUDE::computeDamping(
    const VehicleState& state, MultirotorUDEState* err) const {
  Eigen::Vector3d ude_damping =
      params_->ude_gain * params_->vehicle_mass * state.twist.linear;
  if (err) {
    err->damping_term = ude_damping;
    err->dynamical_term.setConstant(-1.0);
  }
  return ude_damping;
}

Eigen::Vector3d BodyVelocityBasedMultirotorUDE::computeIntegrand(
    const VehicleState& state, const VehicleInput& input,
    MultirotorUDEState* err) const {
  const auto& [velocity, body_rate] = state.twist;
  // omega x vb
  const Eigen::Vector3d dynamical_term =
      body_rate.cross(velocity) * params_->vehicle_mass;

  // The expected thrust/acc
  // f = [0, 0, 1] * thrust
  const double scalar_thrust = input.thrust_attitude().thrust;
  const Eigen::Vector3d expected_thrust =
      Eigen::Vector3d::UnitZ() * scalar_thrust;

  // gravity in body frame
  const Eigen::Vector3d body_gravity =
      state.pose.orientation.inverse() * kGravity;
  const Eigen::Vector3d vehicle_weight = params_->vehicle_mass * body_gravity;

  if (err) {
    err->actuation_term = expected_thrust;
  }

  return dynamical_term - expected_thrust + vehicle_weight - ude_value_;
}

Eigen::Vector3d BodyVelocityBasedMultirotorUDE::computeDamping(
    const VehicleState& state, MultirotorUDEState* err) const {
  const Eigen::Vector3d velocity_body =
      state.pose.orientation.inverse() * state.twist.linear;
  const Eigen::Vector3d ude_damping =
      params_->ude_gain * params_->vehicle_mass * velocity_body;
  if (err) {
    err->damping_term = ude_damping;
    err->dynamical_term.setConstant(-1.0);
  }
  return ude_damping;
}
}  // namespace fsc
