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

#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_APM_ATTITUDE_CONTROLLER_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_APM_ATTITUDE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/math/math_extras.hpp"

namespace fsc {
struct APMAttitudeControllerParams final : public ParameterBase {
  using ParameterBase::load;

  [[nodiscard]] double slew_yaw_max() const { return ang_vel_max.z(); }

  [[nodiscard]] bool valid(LoggerBase& logger) const override;

  [[nodiscard]] std::string parameterFor() const override {
    return "apm_attitude_controller";
  }
  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase& logger) override;

  static constexpr double kDefaultAngleP = 4.5;

  // default maximum acceleration for roll/pitch axis in centidegrees/sec/sec
  static constexpr double kMaxRollPitchAccelDefaultCdss = 110000.0;

  // default maximum acceleration for yaw axis in centidegrees/sec/sec
  static constexpr double kMaxYawAccelDefaultCdss = 27000.0;

  static constexpr double kDefaultYawRateP = 0.180;

  bool rate_bf_ff_enabled{true};
  bool use_sqrt_controller{true};
  double input_tc{1.0};
  double kp_yawrate{kDefaultYawRateP};

  Eigen::Vector3d kp_angle{Eigen::Vector3d::Constant(kDefaultAngleP)};
  Eigen::Vector3d ang_accel_max{deg2rad(kMaxRollPitchAccelDefaultCdss / 100.0),
                                deg2rad(kMaxRollPitchAccelDefaultCdss / 100.0),
                                deg2rad(kMaxYawAccelDefaultCdss / 100.0)};
  Eigen::Vector3d ang_vel_max{Eigen::Vector3d::Zero()};
};

// Thrust angle error above which yaw corrections are limited

class APMAttitudeController : public AttitudeControllerBase {
 public:
  using Parameters = APMAttitudeControllerParams;
  APMAttitudeController() = default;

  struct SetpointAndError {
    Eigen::Vector3d setpoint;
    Eigen::Vector3d error;
  };

  AttitudeControlResult run(const VehicleState& state,
                            const AttitudeReference& refs, double dt,
                            [[maybe_unused]] ContextBase* error) override;

  SetpointAndError attitudeControllerRunQuat(
      const Eigen::Quaterniond& orientation, const Eigen::Vector3d& body_rates,
      double dt);

  [[nodiscard]] std::string name() const override {
    return "apm_attitude_controller";
  }

  bool setParams(const ParameterBase& params, LoggerBase& logger) override;

  [[nodiscard]] std::shared_ptr<ParameterBase> getParams(
      bool use_default) const override;

 private:
  [[nodiscard]] Eigen::Vector3d updateAngVelTargetFromAttError(
      const Eigen::Vector3d& attitude_error_rot_vec_rad, double dt) const;
  bool rate_bf_ff_enabled_{true};
  bool use_sqrt_controller_{true};
  double input_tc_{1.0};
  double kp_yawrate_;
  Eigen::Vector3d kp_angle_;
  Eigen::Vector3d ang_accel_max_;
  Eigen::Vector3d ang_vel_max_{Eigen::Vector3d::Zero()};

  double feedforward_scalar_{0.0};
  Eigen::Quaterniond attitude_target_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d ang_vel_target_{Eigen::Vector3d::Zero()};
  bool parameters_valid_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_APM_ATTITUDE_CONTROLLER_HPP_
