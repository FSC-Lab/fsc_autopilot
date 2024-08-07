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

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"

namespace fsc {

struct NonlinearGeometricControllerParameters : public ParameterBase {
  double time_constant{1.0};

  [[nodiscard]] bool valid(LoggerBase& logger) const override;

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string parameterFor() const override;

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase& logger) override;
};

class NonlinearGeometricController : public AttitudeControllerBase {
 public:
  using Parameters = NonlinearGeometricControllerParameters;

  NonlinearGeometricController() = default;

  AttitudeControlResult run(const VehicleState& state,
                            const AttitudeReference& refs, double dt,
                            ContextBase* error) override;

  [[nodiscard]] std::string name() const final {
    return "simple_attitude_controller";
  }

  bool setParams(const ParameterBase& params, LoggerBase& logger) override;

  [[nodiscard]] std::shared_ptr<ParameterBase> getParams(
      bool use_default) const override;

 private:
  double time_constant_{1.0};
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
