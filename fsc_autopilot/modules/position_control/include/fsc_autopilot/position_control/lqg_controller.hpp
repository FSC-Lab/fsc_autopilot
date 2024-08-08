#ifndef FSC_AUTOPILOT_POSITION_CONTROL_LQG_CONTROLLER_HPP_
#define FSC_AUTOPILOT_POSITION_CONTROL_LQG_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/lqg/lqg_base.hpp"
#include "fsc_autopilot/position_control/control.hpp"
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
  LQGState lqg_state;
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

  uint32_t num_of_rotors{4};

  std::string ude_type;
  UDEParameters ude_params;

  std::string lqg_type;
  LQGParameters lqg_params;

  Eigen::Vector3d ekf_origin;

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

class LQGKF {
 public:
  static constexpr int kStateSize = 7;
  static constexpr int kInputSize = 4;
  using SystemMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using InputMatrix = Eigen::Matrix<double, kStateSize, kInputSize>;
  using State = Eigen::Matrix<double, kStateSize, 1>;
  using Input = Eigen::Matrix<double, kInputSize, 1>;

  LQGKF() = default;
  LQGKF(double mass, const State& x_hat_init);

  State updateKF(const State& x, const Input& control_input_u, double dt);

  bool& lqg_active() { return lqg_active_; }
  [[nodiscard]] bool lqg_active() const { return lqg_active_; }

  bool initialize(double mass, const State& x_hat_init);

 private:
  bool initialized_{false};
  double mass_;  // Set this to the appropriate mass of the vehicle
  bool lqg_active_{true};
  SystemMatrix A_;
  InputMatrix B_;
  SystemMatrix L_;

  State x_hat_;
};

class LQGController final : public ControllerBase {
 public:
  using ParametersSharedPtr = std::shared_ptr<LQGControllerParameters>;
  using ParametersConstSharedPtr =
      std::shared_ptr<const LQGControllerParameters>;

  using UDEConstSharedPtr = std::shared_ptr<const UDEBase>;
  using UDESharedPtr = std::shared_ptr<UDEBase>;

  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  LQGController() = default;

  ControlResult run(const VehicleState& state, const Reference& refs, double dt,
                    ContextBase* error) override;

  bool setParams(const ParameterBase& params, LoggerBase* logger) override;

  void toggleIntegration(bool value) override;

  [[nodiscard]] std::string name() const final { return "lqg_controller"; }

 private:
  double scalar_thrust_setpoint_{0.0};
  bool params_valid_;

  std::uint32_t num_rotors_;
  bool apply_pos_err_saturation_;
  bool apply_vel_err_saturation_;
  double vehicle_mass_;
  double max_tilt_angle_;
  Eigen::Vector3d k_pos_;
  Eigen::Vector3d k_vel_;
  ThrustBounds<double> thrust_bnds_;
  UDESharedPtr ude_;
  LQGKF kf_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_POSITION_CONTROL_LQG_CONTROLLER_HPP_
