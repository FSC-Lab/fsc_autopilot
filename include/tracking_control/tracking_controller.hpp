#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Dense"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/definitions.hpp"
#include "tracking_control/logging.hpp"
#include "tracking_control/ude/ude_base.hpp"

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
                          LoggerBase* logger) override {
    std::ignore =
        loader.getParam("apply_pos_err_saturation", apply_pos_err_saturation);
    k_pos <<  // Force a line break
        loader.param("k_pos/x", kDefaultKpXY),
        loader.param("k_pos/y", kDefaultKpXY),
        loader.param("k_pos/z", kDefaultKpZ);

    std::ignore =
        loader.getParam("apply_vel_err_saturation", apply_vel_err_saturation);
    k_vel <<  // Force a line break
        loader.param("k_vel/x", kDefaultKvXY),
        loader.param("k_vel/y", kDefaultKvXY),
        loader.param("k_vel/z", kDefaultKvZ);

    if (!loader.getParam("vehicle_mass", vehicle_mass)) {
      if (logger) {
        logger->log(Severity::kError,
                    "Failed to load parameter `vehicle_mass`");
      }
      return false;
    }

    std::ignore = loader.getParam("min_thrust", min_thrust);

    std::ignore = loader.getParam("max_thrust", max_thrust);

    std::ignore = loader.getParam("max_tilt_angle", max_tilt_angle);
    return true;
  }

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

#endif  // TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
