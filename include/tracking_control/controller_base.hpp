#ifndef TRACKING_CONTROL_CONTROLLER_BASE_HPP_
#define TRACKING_CONTROL_CONTROLLER_BASE_HPP_

#include <memory>
#include <string>
#include <variant>

#include "Eigen/Dense"
#include "tracking_control/definitions.hpp"

namespace fsc {

struct VehicleInput {
  double thrust;
  std::variant<Eigen::Quaterniond, Eigen::Vector3d> command;

  Eigen::Quaterniond& orientation() {
    return std::get<Eigen::Quaterniond>(command);
  }

  [[nodiscard]] const Eigen::Quaterniond& orientation() const {
    return std::get<Eigen::Quaterniond>(command);
  }

  Eigen::Vector3d& body_rates() { return std::get<Eigen::Vector3d>(command); }

  [[nodiscard]] const Eigen::Vector3d& body_rates() const {
    return std::get<Eigen::Vector3d>(command);
  }
};

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

  [[nodiscard]] virtual std::string name() const = 0;
};

struct ControlResult {
  bool success{false};
  Setpoint setpoint;
  std::shared_ptr<ControlErrorBase> error;
  std::string message;
};

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual ControlResult run(const VehicleState& state,
                            const Reference& refs) = 0;

  [[nodiscard]] virtual Setpoint getFallBackSetpoint() const {
    return {VehicleState{}, VehicleInput{0.0, Eigen::Quaterniond::Identity()}};
  }

  [[nodiscard]] virtual std::string name() const = 0;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_CONTROLLER_BASE_HPP_
