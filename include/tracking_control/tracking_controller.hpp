#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "tracking_control/controller_base.hpp"
#include "utils/utils.hpp"

namespace fsc {

struct TrackingControllerError final : public ControlErrorBase {
  [[nodiscard]] std::string name() const final {
    return "tracking_controller.error";
  }

  std::string msg_str;
  [[nodiscard]] std::string message() const final { return msg_str; }

  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ude_output{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_sp{Eigen::Vector3d::Zero()};

  // msg output
  bool altiThreshold{false};
  bool intFlag{false};
  float thrustPerRotor;  // thrust per rotor
  float thrust_sp;       // thrust setpoint
  Eigen::Vector3d expectedThrust{Eigen::Vector3d::Zero()};
  Eigen::Vector3d inertialForce{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbanceEstimate{Eigen::Vector3d::Zero()};

  bool ude_effective{false};
};

struct TrackingControllerParameters {
  static constexpr double kDefaultKpXY{1.0};
  static constexpr double kDefaultKpZ{10.0};
  Eigen::Vector3d k_pos{kDefaultKpXY, kDefaultKpXY, kDefaultKpZ};

  static constexpr double kDefaultKvXY{1.5};
  static constexpr double kDefaultKvZ{3.3};
  Eigen::Vector3d k_vel{kDefaultKvXY, kDefaultKvXY, kDefaultKvZ};
  double min_z_accel{0};

  static constexpr double kDefaultMaxZAccel{20};
  double max_z_accel{kDefaultMaxZAccel};

  static constexpr double kDefaultMaxTiltAngle{45};
  double max_tilt_angle{kDefaultMaxTiltAngle};

  static constexpr double kDefaultDEGain{1.0};
  double de_gain{kDefaultDEGain};

  static constexpr double kDefaultDEHeightThreshold{0.1};
  double de_height_threshold{kDefaultDEHeightThreshold};

  static constexpr double kVehicleMassSentinel{-1.0};
  double vehicle_mass{kVehicleMassSentinel};

  uint32_t num_of_rotors{4};

  bool ude_active{false};
  bool ude_is_velocity_based{false};

  static constexpr double kDefaultDEBounds{5};
  Eigen::Vector3d de_lb{Eigen::Vector3d::Constant(-kDefaultDEBounds)};
  Eigen::Vector3d de_ub{Eigen::Vector3d::Constant(kDefaultDEBounds)};

  double dt{-1.0};
};

class TrackingController final : public ControllerBase {
 public:
  using ParametersSharedPtr = std::shared_ptr<TrackingControllerParameters>;
  using ParametersConstSharedPtr =
      std::shared_ptr<const TrackingControllerParameters>;

  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  TrackingController() = default;
  explicit TrackingController(ParametersSharedPtr params);

  ControlResult run(const VehicleState& state, const Reference& refs,
                    ControlErrorBase* error) override;

  [[nodiscard]] ParametersConstSharedPtr params() const { return params_; }
  ParametersSharedPtr& params() { return params_; }

  [[nodiscard]] std::string name() const final { return "tracking_controller"; }

 private:
  Eigen::Vector3d disturbance_estimate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d de_integral_{Eigen::Vector3d::Zero()};

  double thrust_sp_{0.0};
  ParametersSharedPtr params_;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
