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
#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"

namespace fsc {
struct TrackingControllerError : public ContextBase {
  [[nodiscard]] std::string name() const final {
    return "tracking_controller.error";
  }

  bool int_flag{false};

  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d feedback{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_setpoint{Eigen::Vector3d::Zero()};
  double scalar_thrust_sp{0.0};  // thrust setpoint
  double thrust_per_rotor{0.0};  // thrust per rotor
  UDEState ude_state;
};

struct TrackingControllerParameters : public ParameterBase {
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

  uint32_t num_of_rotors{4};

  [[nodiscard]] bool valid() const override {
    return min_thrust < max_thrust && vehicle_mass > 0.0;
  }

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase* logger) override;

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string parameterFor() const override {
    return "tracking_controller";
  }
};

class TrackingController final : public ControllerBase {
 public:
  using ParametersSharedPtr = std::shared_ptr<TrackingControllerParameters>;
  using ParametersConstSharedPtr =
      std::shared_ptr<const TrackingControllerParameters>;

  using UDEConstSharedPtr = std::shared_ptr<const UDEBase>;
  using UDESharedPtr = std::shared_ptr<UDEBase>;

  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  TrackingController() = default;
  explicit TrackingController(ParametersSharedPtr params);

  TrackingController(ParametersSharedPtr params, UDESharedPtr ude);

  ControlResult run(const VehicleState& state, const Reference& refs,
                    ContextBase* error) override;

  [[nodiscard]] ParametersConstSharedPtr params() const { return params_; }
  ParametersSharedPtr& params() { return params_; }

  [[nodiscard]] UDEConstSharedPtr ude() const { return ude_; }
  UDESharedPtr& ude() { return ude_; }

  [[nodiscard]] std::string name() const final { return "tracking_controller"; }

 private:
  double scalar_thrust_setpoint_{0.0};
  ParametersSharedPtr params_;
  UDESharedPtr ude_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_POSITION_CONTROL_TRACKING_CONTROLLER_HPP_
