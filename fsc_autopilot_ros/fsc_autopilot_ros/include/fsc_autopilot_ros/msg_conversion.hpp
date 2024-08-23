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

#include <fsc_autopilot/math/math_extras.hpp>
#include <fsc_autopilot/position_control/position_controller_base.hpp>

#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot_msgs/AttitudeControllerState.h"
#include "fsc_autopilot_msgs/PositionControllerReference.h"
#include "fsc_autopilot_msgs/PositionControllerState.h"
#include "fsc_autopilot_msgs/UDEState.h"
#include "tf2_eigen/tf2_eigen.h"

namespace fsc {

inline fsc_autopilot_msgs::AttitudeControllerState& toMsg(
    const tf2::Stamped<fsc::AttitudeControllerState>& in,
    fsc_autopilot_msgs::AttitudeControllerState& out) {
  out.header.stamp = in.stamp_;
  out.reference = tf2::toMsg(in.reference);
  out.feedback = tf2::toMsg(in.feedback);
  out.attitude_error = tf2::toMsg(in.attitude_error);
  tf2::toMsg(in.error, out.error);
  tf2::toMsg(in.rate_feedforward, out.rate_feedforward);
  tf2::toMsg(in.output, out.output);
  return out;
}

inline fsc_autopilot_msgs::UDEState& toMsg(const fsc::UDEState& in,
                                           fsc_autopilot_msgs::UDEState& out) {
  out.type = in.type_str;
  out.is_flying = static_cast<std::uint8_t>(in.is_flying);
  out.is_active = static_cast<std::uint8_t>(in.is_active);
  tf2::toMsg(in.damping_term, out.damping_term);
  tf2::toMsg(in.dynamical_term, out.dynamical_term);
  tf2::toMsg(in.actuation_term, out.actuation_term);
  tf2::toMsg(in.integral, out.integral);
  tf2::toMsg(in.disturbance_estimate, out.disturbance_estimate);
  return out;
}

inline void fromMsg(const fsc_autopilot_msgs::PositionControllerReference& in,
                    tf2::Stamped<PositionControllerReference>& out) {
  out.stamp_ = in.header.stamp;
  tf2::fromMsg(in.position, out.position);
  tf2::fromMsg(in.velocity, out.velocity);
  tf2::fromMsg(in.acceleration, out.acceleration);
  tf2::fromMsg(in.thrust, out.thrust);
  const bool convert_degrees =
      in.yaw_unit == fsc_autopilot_msgs::PositionControllerReference::DEGREES;
  out.yaw = convert_degrees ? deg2rad(in.yaw) : in.yaw;
}

inline fsc_autopilot_msgs::PositionControllerState& toMsg(
    const tf2::Stamped<fsc::PositionControllerState>& in,
    fsc_autopilot_msgs::PositionControllerState& out) {
  out.header.stamp = in.stamp_;

  out.position_reference = tf2::toMsg(in.position_reference);
  tf2::toMsg(in.velocity_reference, out.velocity_reference);

  out.position = tf2::toMsg(in.position);
  tf2::toMsg(in.velocity, out.velocity);
  tf2::toMsg(in.acceleration, out.acceleration);

  tf2::toMsg(in.position_error, out.position_error);
  tf2::toMsg(in.velocity_error, out.velocity_error);

  tf2::toMsg(in.output, out.output);
  return out;
}

}  // namespace fsc

#endif  // FSC_AUTOPILOT_ROS_MSG_CONVERSION_HPP_
