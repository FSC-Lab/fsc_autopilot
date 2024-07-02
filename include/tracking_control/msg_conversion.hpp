#ifndef TRACKING_CONTROL_MSG_CONVERSION_HPP_
#define TRACKING_CONTROL_MSG_CONVERSION_HPP_

#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/TrackingError.h"
#include "tracking_control/UDEState.h"
#include "tracking_control/tracking_controller.hpp"
#include "tracking_control/ude/multirotor_ude.hpp"
namespace tf2 {

inline tracking_control::UDEState& toMsg(const fsc::MultirotorUDEState& in,
                                         tracking_control::UDEState& out) {
  switch (in.type) {
    case fsc::MultirotorUDEType::kVelocityBased:
      out.type = "velocity based";
      break;
    case fsc::MultirotorUDEType::kAccelBased:
      out.type = "acceleration based";
      break;
    case fsc::MultirotorUDEType::kBodyVelocityBased:
      out.type = "body velocity based";
      break;
    case fsc::MultirotorUDEType::kBodyAccelBased:
      out.type = "body acceleration based";
      break;
  }

  out.is_flying = static_cast<std::uint8_t>(in.is_flying);
  out.is_active = static_cast<std::uint8_t>(in.is_active);
  toMsg(in.damping_term, out.damping_term);
  toMsg(in.dynamical_term, out.dynamical_term);
  toMsg(in.actuation_term, out.actuation_term);
  toMsg(in.integral, out.integral);
  toMsg(in.disturbance_estimate, out.disturbance_estimate);
  return out;
}

inline tracking_control::TrackingError& toMsg(
    const Stamped<fsc::TrackingControllerError>& in,
    tracking_control::TrackingError& out) {
  out.header.stamp = in.stamp_;
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

#endif  // TRACKING_CONTROL_MSG_CONVERSION_HPP_
