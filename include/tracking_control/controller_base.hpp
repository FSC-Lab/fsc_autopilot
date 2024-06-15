#ifndef TRACKING_CONTROL_CONTROLLER_BASE_HPP_
#define TRACKING_CONTROL_CONTROLLER_BASE_HPP_

#include <memory>
#include <string>
#include <variant>

#include "Eigen/Dense"

namespace fsc {

struct Pose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct Accel {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct VehicleState {
  double stamp;
  Pose pose;
  Twist twist;
  Accel accel;
};

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

  template <typename T>
  std::shared_ptr<T> errorAs(const std::string& name) {
    if (error->name() == name) {
      return std::static_pointer_cast<T>(error);
    }
    return nullptr;
  }
};

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual ControlResult run(const VehicleState& state, const Setpoint& refs,
                            double dt) = 0;

  [[nodiscard]] virtual std::string name() const = 0;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_CONTROLLER_BASE_HPP_
