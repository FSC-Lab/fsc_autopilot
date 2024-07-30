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

#ifndef FSC_AUTOPILOT_POSITION_CONTROL_TRACKING_CONTROLLER_HPP_
#define FSC_AUTOPILOT_POSITION_CONTROL_TRACKING_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/position_control/control.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"

namespace fsc {
struct RobustControllerParameters : public ParameterBase {
  using ParameterBase::load;
  static constexpr double kDefaultMaxThrust{20};     // In Newtons
  static constexpr double kDefaultMaxTiltAngle{45};  // In Degrees
  static constexpr double kDefaultKpXY{1.5};
  static constexpr double kDefaultKpZ{1.0};
  static constexpr double kDefaultKvXY{5};
  static constexpr double kDefaultKvZ{3};

  bool apply_pos_err_saturation{true};
  bool apply_vel_err_saturation{false};
  double min_thrust{0};
  double max_thrust{kDefaultMaxThrust};
  double max_tilt_angle{kDefaultMaxTiltAngle};
  double vehicle_mass{-1.0};
  PIDForm form;
  Eigen::Vector3d k_pos{kDefaultKpXY, kDefaultKpXY, kDefaultKpZ};
  Eigen::Vector3d k_vel{kDefaultKvXY, kDefaultKvXY, kDefaultKvZ};

  [[nodiscard]] bool valid(LoggerBase& logger) const override;

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase& logger) override;

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string parameterFor() const override {
    return "robust_controller";
  }
};

class RobustController final : public PositionControllerBase {
 public:
  using Parameters = RobustControllerParameters;

  RobustController() = default;

  PositionControlResult run(const VehicleState& state,
                            const PositionControllerReference& refs, double dt,
                            ContextBase* error) override;

  bool setParams(const ParameterBase& params, LoggerBase& logger) override;

  [[nodiscard]] std::shared_ptr<ParameterBase> getParams(
      bool use_default) const override;

  [[nodiscard]] std::string name() const final { return "robust_controller"; }

 private:
  bool apply_pos_err_saturation_{true};
  bool apply_vel_err_saturation_{false};

  // This is a 'safe' default; If users forgot to set this parameter, then the
  // default mass 1kg will be unrealistically small, and their drones simply
  // can't take off
  double vehicle_mass_{1.0};
  double max_tilt_angle_{0.0};
  PIDForm form_{PIDForm::kIdeal};
  Eigen::Vector3d k_pos_{Parameters::kDefaultKpXY, Parameters::kDefaultKpXY,
                         Parameters::kDefaultKpZ};
  Eigen::Vector3d k_vel_{Parameters::kDefaultKvXY, Parameters::kDefaultKvXY,
                         Parameters::kDefaultKpZ};
  ThrustBounds<double> thrust_bnds_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_POSITION_CONTROL_TRACKING_CONTROLLER_HPP_
