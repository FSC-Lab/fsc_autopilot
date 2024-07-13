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

#include "fsc_autopilot/attitude_control/nonlinear_geometric_controller.hpp"

#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/math/rotation.hpp"

namespace fsc {

NonlinearGeometricController::NonlinearGeometricController(Parameters params)
    : params_(params) {}

ControlResult NonlinearGeometricController::run(
    const VehicleState& state, const Reference& refs,
    [[maybe_unused]] ContextBase* error) {
  const auto rotmat = state.pose.orientation.toRotationMatrix();
  const auto rotmat_sp = refs.state.pose.orientation.toRotationMatrix();

  NonlinearGeometricControllerError* err;
  if ((error != nullptr) &&
      error->name() == "nonlinear_geometric_controller.error") {
    err = static_cast<decltype(err)>(error);
  }
  // e_r = 1 / 2 * (Rd.T * R - R.T * Rd)
  err->attitude_error = fsc::vee(rotmat_sp.transpose() * rotmat -
                                 rotmat.transpose() * rotmat_sp) /
                        2;

  const Eigen::Vector3d body_rate_sp =
      -2.0 / params_.time_constant * err->attitude_error;

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input = VehicleInput{-1.0, body_rate_sp};
  return result;
}
}  // namespace fsc
