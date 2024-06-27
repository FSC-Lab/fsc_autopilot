#ifndef TRACKING_CONTROL_TF2_EXTRAS_HPP_
#define TRACKING_CONTROL_TF2_EXTRAS_HPP_

#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/TrackingError.h"
#include "tracking_control/UDEState.h"
#include "tracking_control/tracking_controller.hpp"
namespace tf2 {

inline tracking_control::UDEState& toMsg(
    const fsc::TrackingControllerError::UDEState& in,
    tracking_control::UDEState& out) {
  out.is_velocity_based = static_cast<std::uint8_t>(in.is_velocity_based);
  toMsg(in.damping, out.damping);
  toMsg(in.inertial_force, out.inertial_force);
  toMsg(in.expected_thrust, out.expected_thrust);
  toMsg(in.integral, out.integral);
  toMsg(in.disturbance_estimate, out.disturbance_estimate);
  return out;
}

inline tracking_control::TrackingError& toMsg(
    const Stamped<fsc::TrackingControllerError>& in,
    tracking_control::TrackingError& out) {
  out.int_flag = static_cast<std::uint8_t>(in.int_flag);
  out.pass_alt_threshold = static_cast<std::uint8_t>(in.pass_alt_threshold);
  out.scalar_thrust_setpoint = in.scalar_thrust_sp;
  out.thrust_per_rotor = in.thrust_per_rotor;
  toMsg(in.position_error, out.position_error);
  toMsg(in.velocity_error, out.velocity_error);
  toMsg(in.feedback, out.feedback);
  toMsg(in.thrust_setpoint, out.thrust_setpoint);
  toMsg(in.ude_state, out.ude_state);
  return out;
}
}  // namespace tf2

#endif  // TRACKING_CONTROL_TF2_EXTRAS_HPP_
