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

#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_

#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/core/controller_base.hpp"

namespace fsc {

struct NonlinearGeometricControllerError final : public ContextBase {
  Eigen::Vector3d attitude_error;

  [[nodiscard]] std::string name() const final {
    return "nonlinear_geometric_controller.error";
  }
};

struct NonlinearGeometricControllerParameters {
  double time_constant;
};

struct NonlinearGeometricController : public ControllerBase {
  using Parameters = NonlinearGeometricControllerParameters;
  using Error = NonlinearGeometricControllerError;

  NonlinearGeometricController() = default;

  explicit NonlinearGeometricController(Parameters params);

  ControlResult run(const VehicleState& state, const Reference& refs,
                    ContextBase* error) override;

  [[nodiscard]] const Parameters& params() const { return params_; }

  Parameters& params() { return params_; }

  [[nodiscard]] std::string name() const final {
    return "nonlinear_geometric_controller";
  }

 private:
  Parameters params_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
