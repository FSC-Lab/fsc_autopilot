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

#ifndef FSC_AUTOPILOT_ROS_MSG_CONVERSION_HPP_
#define FSC_AUTOPILOT_ROS_MSG_CONVERSION_HPP_

#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/position_control/tracking_controller.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot_msgs/AttitudeControllerState.h"
#include "fsc_autopilot_msgs/TrackingError.h"
#include "fsc_autopilot_msgs/UDEState.h"
#include "tf2_eigen/tf2_eigen.h"

namespace tf2 {

inline fsc_autopilot_msgs::AttitudeControllerState& toMsg(
    const Stamped<fsc::AttitudeControllerState>& in,
    fsc_autopilot_msgs::AttitudeControllerState& out) {
  out.header.stamp = in.stamp_;
  out.reference = tf2::toMsg(in.reference);
  out.feedback = tf2::toMsg(in.feedback);
  out.attitude_error = tf2::toMsg(in.attitude_error);
  toMsg(in.error, out.error);
  toMsg(in.rate_feedforward, out.rate_feedforward);
  toMsg(in.output, out.output);
  return out;
}

inline fsc_autopilot_msgs::UDEState& toMsg(const fsc::UDEState& in,
                                           fsc_autopilot_msgs::UDEState& out) {
  out.type = in.type_str;
  out.is_flying = static_cast<std::uint8_t>(in.is_flying);
  out.is_active = static_cast<std::uint8_t>(in.is_active);
  toMsg(in.damping_term, out.damping_term);
  toMsg(in.dynamical_term, out.dynamical_term);
  toMsg(in.actuation_term, out.actuation_term);
  toMsg(in.integral, out.integral);
  toMsg(in.disturbance_estimate, out.disturbance_estimate);
  return out;
}

inline fsc_autopilot_msgs::TrackingError& toMsg(
    const Stamped<fsc::TrackingControllerError>& in,
    fsc_autopilot_msgs::TrackingError& out) {
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

#endif  // FSC_AUTOPILOT_ROS_MSG_CONVERSION_HPP_
