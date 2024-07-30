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

#include "fsc_autopilot/core/vehicle_model.hpp"

#include <iostream>
#include <set>
#include <vector>

#include "fsc_autopilot/core/logger_base.hpp"

namespace fsc {

bool VehicleModelParameters::load(const ParameterLoaderBase& loader,
                                  LoggerBase& logger) {
  if (!loader.getParam("name", vehicle_name)) {
    logger.log(Severity::kError, "Failed to get required_parameter: `name`");
  }
  std::vector<double> motor_curve_coeffs_params;
  if (!loader.getParam("motor_curve", motor_curve_coeffs_params)) {
    logger.log(Severity::kError,
               "Failed to get required parameter: `motor_curve`");
    return false;
  }

  motor_curve_coeffs = Eigen::VectorXd::Map(
      motor_curve_coeffs_params.data(),
      static_cast<Eigen::Index>(motor_curve_coeffs_params.size()));

  if (!loader.getParam("num_rotors", num_rotors)) {
    logger.log(Severity::kError,
               "Failed to get required parameter: `num_rotors`");
    return false;
  }
  return true;
}

bool VehicleModelParameters::valid(LoggerBase& logger) const {
  if (vehicle_name.empty()) {
    logger.log(Severity::kError,
               "Vehicle must be identified by a valid, non-empty name");
    return false;
  }

  std::set<int> rotor_set{3, 4, 6, 8};
  if (rotor_set.count(num_rotors) == 0) {
    logger.log(Severity::kError, "Number of rotors must in the set {3,4,6,8}");
    return false;
  }

  if (motor_curve_coeffs.size() <= 0L) {
    logger.log(Severity::kError, "Motor curve must be at least a monomial");
    return false;
  }
  return true;
}

std::string VehicleModelParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};

  std::ostringstream oss;
  oss << "Vehicle Model parameters:\n - num_rotors: " << num_rotors
      << "\n - motor_curve coefficients:"
      << motor_curve_coeffs.transpose().format(f);
  return oss.str();
}

VehicleInput VehicleModel::transformInputs(const VehicleInput& input) const {
  return std::visit(
      [this](auto arg) {
        arg.thrust =
            std::clamp(motor_curve.vals(arg.thrust / num_rotors), 0.0, 1.0);
        return VehicleInput{arg};
      },
      input.command());
}

VehicleModelParameters VehicleModel::getParams() const {
  VehicleModelParameters params;
  params.vehicle_name = vehicle_name;
  params.num_rotors = num_rotors;
  params.motor_curve_coeffs = motor_curve.coeffs();
  return params;
}

bool VehicleModel::setParams(const VehicleModelParameters& params,
                             LoggerBase& logger) {
  if (!params.valid(logger)) {
    return false;
  }
  vehicle_name = params.vehicle_name;
  num_rotors = params.num_rotors;
  motor_curve = MotorCurveType(params.motor_curve_coeffs);
  return true;
}

}  // namespace fsc
