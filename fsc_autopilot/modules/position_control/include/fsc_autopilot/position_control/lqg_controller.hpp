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

#ifndef FSC_AUTOPILOT_POSITION_CONTROL_LQG_CONTROLLER_HPP_
#define FSC_AUTOPILOT_POSITION_CONTROL_LQG_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/position_control/control.hpp"

namespace fsc {
struct LQGControllerError : public ContextBase {
  [[nodiscard]] std::string name() const final {
    return "lqg_controller.error";
  }

  bool int_flag{false};

  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d feedback{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_setpoint{Eigen::Vector3d::Zero()};
  double scalar_thrust_sp{0.0};  // thrust setpoint
  double thrust_per_rotor{0.0};  // thrust per rotor
};

struct LQGControllerParameters : public ParameterBase {
  using ParameterBase::load;

  bool apply_pos_err_saturation{true};
  static constexpr double kDefaultKpXY{1.0};
  static constexpr double kDefaultKpZ{10.0};
  Eigen::Vector3d k_pos{kDefaultKpXY, kDefaultKpXY, kDefaultKpZ};

  bool apply_vel_err_saturation{false};
  static constexpr double kDefaultKvXY{1.5};
  static constexpr double kDefaultKvZ{3.3};
  Eigen::Vector3d k_vel{kDefaultKvXY, kDefaultKvXY, kDefaultKvZ};
  double min_thrust{0};

  static constexpr double kDefaultMaxThrust{20};
  double max_thrust{kDefaultMaxThrust};

  static constexpr double kDefaultMaxTiltAngle{45};
  double max_tilt_angle{kDefaultMaxTiltAngle};

  static constexpr double kVehicleMassSentinel{-1.0};
  double vehicle_mass{kVehicleMassSentinel};

  std::string ude_type;

  [[nodiscard]] bool valid(LoggerBase& logger) const override;

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase& logger) override;

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string parameterFor() const override {
    return "lqg_controller";
  }
};

class LQGController final : public ControllerBase {
 public:
  using ParametersSharedPtr = std::shared_ptr<LQGControllerParameters>;
  using ParametersConstSharedPtr =
      std::shared_ptr<const LQGControllerParameters>;

  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  LQGController() = default;

  ControlResult run(const VehicleState& state, const Reference& refs, double dt,
                    ContextBase* error) override;

  bool setParams(const ParameterBase& params, LoggerBase& logger) override;

  [[nodiscard]] std::shared_ptr<ParameterBase> getParams(
      bool use_default) const override;

  void toggleIntegration(bool value) override;
  Eigen::Matrix<double, 6, 1> getEstimatedState() const;

  [[nodiscard]] std::string name() const final { return "lqg_controller"; }

 private:
  double scalar_thrust_setpoint_{0.0};
  bool params_valid_;

  bool apply_pos_err_saturation_{true};
  bool apply_vel_err_saturation_{false};
  bool kf_reset_{false};
  double vehicle_mass_;
  double max_tilt_angle_;
  Eigen::Vector3d k_pos_;
  Eigen::Vector3d k_vel_;
  ThrustBounds<double> thrust_bnds_;
  Eigen::Vector3d estimated_position_;
  Eigen::Vector3d estimated_velocity_;
  Eigen::Matrix<double, 6, 1> estimated_state_;
  Eigen::Matrix<double, 12, 1> H_infinity_state_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_POSITION_CONTROL_LQG_CONTROLLER_HPP_
