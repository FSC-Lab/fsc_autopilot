
#include <memory>
#include <string>

#include "Eigen/Dense"
#include "tracking_control/control.hpp"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/core/logger_base.hpp"
#include "tracking_control/core/parameter_base.hpp"
#include "tracking_control/definitions.hpp"
#include "tracking_control/math.hpp"

namespace fsc {
struct APMAttitudeControllerParams final : public ParameterBase {
  using ParameterBase::load;

  static constexpr double kDefaultAngleP = 4.5;
  static constexpr double kMaxRollPitchAccelDefaultCdss =
      110000.0;  // default maximum acceleration for roll/pitch axis in
                 // centidegrees/sec/sec
  static constexpr double kMaxYawAccelDefaultCdss =
      27000.0;  // default maximum acceleration for yaw axis in
                // centidegrees/sec/sec
  bool rate_bf_ff_enabled{true};
  bool use_sqrt_controller{true};
  double input_tc{1.0};
  double dt{-1.0};
  Eigen::Vector3d kp_angle{Eigen::Vector3d::Constant(kDefaultAngleP)};
  Eigen::Vector3d ang_accel_max{deg2rad(kMaxRollPitchAccelDefaultCdss / 100.0),
                                deg2rad(kMaxRollPitchAccelDefaultCdss / 100.0),
                                deg2rad(kMaxYawAccelDefaultCdss / 100.0)};
  Eigen::Vector3d ang_vel_max{Eigen::Vector3d::Zero()};

  static constexpr double kDefaultYawRateP = 0.180;
  double kp_yawrate{kDefaultYawRateP};

  [[nodiscard]] double slew_yaw_max() const { return ang_vel_max.z(); }

  [[nodiscard]] bool valid() const override {
    return input_tc > 0 && dt > 0 && (kp_angle.array() >= 0.0).all() &&
           (ang_accel_max.array() >= 0.0).all() &&
           (ang_vel_max.array() >= 0.0).all();
  }

  [[nodiscard]] std::string parameterFor() const override {
    return "apm_attitude_controller";
  }
  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase* logger) override;
};

// Thrust angle error above which yaw corrections are limited

class APMAttitudeController : public ControllerBase {
 public:
  using Parameters = APMAttitudeControllerParams;
  using ParametersSharedPtr = std::shared_ptr<Parameters>;
  APMAttitudeController() = default;
  explicit APMAttitudeController(ParametersSharedPtr params);

  ControlResult run(const VehicleState& state, const Reference& refs,
                    [[maybe_unused]] ContextBase* error) override;

  Eigen::Vector3d attitudeControllerRunQuat(
      const Eigen::Quaterniond& orientation, const Eigen::Vector3d& body_rates);

  [[nodiscard]] std::string name() const override {
    return "apm_attitude_controller";
  }
  [[nodiscard]] const ParametersSharedPtr& params() const { return params_; }

  ParametersSharedPtr& params() { return params_; }

 private:
  [[nodiscard]] Eigen::Vector3d updateAngVelTargetFromAttError(
      const Eigen::Vector3d& attitude_error_rot_vec_rad) const;

  ParametersSharedPtr params_{std::make_shared<Parameters>()};
  double feedforward_scalar_{0.0};
  Eigen::Quaterniond attitude_target_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ang_vel_target_{Eigen::Vector3d::Zero()};
};
}  // namespace fsc
