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

#ifndef FSC_AUTOPILOT_CORE_VEHICLE_MODEL_HPP_
#define FSC_AUTOPILOT_CORE_VEHICLE_MODEL_HPP_

#include <string>

#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/math/polynomial.hpp"

namespace fsc {

class VehicleModelParameters final : public ParameterBase {
 public:
  int num_rotors;
  Eigen::VectorXd motor_curve_coeffs;

  [[nodiscard]] bool valid(LoggerBase& logger) const override;

  [[nodiscard]] std::string parameterFor() const override {
    return "vehicle_model";
  }

  bool load(const ParameterLoaderBase& loader, LoggerBase& logger) override;

  [[nodiscard]] std::string toString() const override;
};

struct VehicleModel {
 public:
  using MotorCurveType = math::Polynomial<double>;

  [[nodiscard]] VehicleInput transformInputs(const VehicleInput& input) const;

  [[nodiscard]] VehicleModelParameters getParams() const;

  bool setParams(const VehicleModelParameters& params, LoggerBase& logger);

  int num_rotors;
  MotorCurveType motor_curve;
};

}  // namespace fsc

#endif  // FSC_AUTOPILOT_CORE_VEHICLE_MODEL_HPP_
