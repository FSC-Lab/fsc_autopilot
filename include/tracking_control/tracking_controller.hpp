#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include <memory>
#include <sstream>
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
  Eigen::Vector3d thrust_setpoint{Eigen::Vector3d::Zero()};

  // msg output
  bool altiThreshold{false};
  bool intFlag{false};
  double thrust_per_rotor;  // thrust per rotor
  double thrust_sp;         // thrust setpoint
  Eigen::Vector3d expectedThrust{Eigen::Vector3d::Zero()};
  Eigen::Vector3d inertialForce{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbanceEstimate{Eigen::Vector3d::Zero()};

  bool ude_effective{false};
};

struct TrackingControllerParameters : public ControllerParameterBase {
  bool is_pos_err_saturation_active{true};
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

  static constexpr double kDefaultDEGain{1.0};
  double ude_gain{kDefaultDEGain};

  static constexpr double kDefaultDEHeightThreshold{0.1};
  double ude_height_threshold{kDefaultDEHeightThreshold};

  static constexpr double kVehicleMassSentinel{-1.0};
  double vehicle_mass{kVehicleMassSentinel};

  uint32_t num_of_rotors{4};

  bool ude_active{false};
  bool ude_is_velocity_based{false};

  static constexpr double kDefaultUDEBounds{5};
  Eigen::Vector3d ude_lb{Eigen::Vector3d::Constant(-kDefaultUDEBounds)};
  Eigen::Vector3d ude_ub{Eigen::Vector3d::Constant(kDefaultUDEBounds)};

  double dt{-1.0};

  [[nodiscard]] bool valid() const override {
    return min_thrust < max_thrust && (ude_lb.array() < ude_ub.array()).all() &&
           vehicle_mass > 0.0 && dt > 0.0;
  }

  [[nodiscard]] std::string toString() const override {
    const Eigen::IOFormat f{
        Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
    std::ostringstream oss;

    oss << "dt: " << dt << "\n"
        << "Quadrotor Mass: " << vehicle_mass << "\n"
        << "thrust bounds: [" << min_thrust << "," << max_thrust << "]\n"
        << "Tracking Controller parameters:\nk_pos: "
        << k_pos.transpose().format(f)                     //
        << "\nk_vel: " << k_vel.transpose().format(f)      //
        << "\nUDE parameters:"                             //
        << "\nheight_threshold: " << ude_height_threshold  //
        << "\nde_gain: " << ude_gain                       //
        << "\nde_lb: " << ude_lb.transpose().format(f)     //
        << "\nde_ub: " << ude_ub.transpose().format(f);    //
    return oss.str();
  }
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
  Eigen::Vector3d ude_value_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ude_integral_{Eigen::Vector3d::Zero()};

  double scalar_thrust_setpoint_{0.0};
  ParametersSharedPtr params_;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
