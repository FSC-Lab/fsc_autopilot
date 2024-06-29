#ifndef TRACKING_CONTROL_CONTROLLER_BASE_HPP_
#define TRACKING_CONTROL_CONTROLLER_BASE_HPP_

#include <string>
#include <system_error>

#include "Eigen/Dense"
#include "tracking_control/definitions.hpp"
#include "tracking_control/vehicle_input.hpp"

namespace fsc {

enum class ReferenceKind {
  kPosition = 1 << 0,
  kAttitude = 1 << 1,
  kVelocity = 1 << 2,
  kBodyRate = 1 << 3
};

struct Reference {
  VehicleState state;
  double yaw;
};

struct Setpoint {
  VehicleState state;
  VehicleInput input;
};

struct ControlErrorBase {
  virtual ~ControlErrorBase() = default;

  [[nodiscard]] virtual std::string message() const { return ""; }

  [[nodiscard]] virtual std::string name() const = 0;
};

struct ControlResult {
  Setpoint setpoint;
  std::error_code ec;

  explicit operator bool() const noexcept {
    return ec == ControllerErrc::kSuccess;
  }
};

class ControllerParameterBase {
 public:
  virtual ~ControllerParameterBase() = default;

  [[nodiscard]] virtual bool valid() const = 0;

  [[nodiscard]] virtual std::string toString() const = 0;
};

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual ControlResult run(const VehicleState& state, const Reference& refs,
                            ControlErrorBase* error) = 0;

  ControlResult run(const VehicleState& state, const Reference& refs) {
    return run(state, refs, nullptr);
  }

  [[nodiscard]] virtual Setpoint getFallBackSetpoint() const {
    return {VehicleState{}, VehicleInput{0.0, Eigen::Quaterniond::Identity()}};
  }

  [[nodiscard]] virtual std::string name() const = 0;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_CONTROLLER_BASE_HPP_
