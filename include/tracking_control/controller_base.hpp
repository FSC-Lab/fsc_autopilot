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

class Context {
 public:
  virtual ~Context() = default;

  virtual bool getScalar([[maybe_unused]] std::string_view name,
                         [[maybe_unused]] double& value) {
    return false;
  }

  virtual bool setScalar([[maybe_unused]] std::string_view name,
                         [[maybe_unused]] double value) {
    return false;
  }

  [[nodiscard]] virtual bool getFlag([[maybe_unused]] std::string_view name,
                                     [[maybe_unused]] bool& value) const {
    return false;
  }

  virtual bool setFlag([[maybe_unused]] std::string_view name,
                       [[maybe_unused]] bool value) {
    return false;
  }

  virtual bool setScalar([[maybe_unused]] std::string_view name,
                         [[maybe_unused]] bool value) {
    return false;
  }

  [[nodiscard]] virtual bool getArray(
      [[maybe_unused]] std::string_view name,
      [[maybe_unused]] Eigen::Ref<Eigen::VectorXd>& x) const {
    return false;
  }

  [[nodiscard]] bool setArray(
      [[maybe_unused]] std::string_view name,
      [[maybe_unused]] const Eigen::Ref<const Eigen::VectorXd>& value) {
    return false;
  }
};

struct VehicleState {
  double stamp;
  Pose pose;
  Twist twist;
  Accel accel;
  std::shared_ptr<Context> ctx;
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

struct ControlErrorBase : public Context {
  ~ControlErrorBase() override = default;

  [[nodiscard]] virtual std::string name() const = 0;
};

struct ControlResult {
  bool success{false};
  Setpoint setpoint;
  std::shared_ptr<ControlErrorBase> error;
};

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual ControlResult run(const VehicleState& state, const Setpoint& refs,
                            double dt) = 0;

  [[nodiscard]] virtual Setpoint getFallBackSetpoint() const {
    return {VehicleState{}, VehicleInput{0.0, Eigen::Quaterniond::Identity()}};
  }

  [[nodiscard]] virtual std::string name() const = 0;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_CONTROLLER_BASE_HPP_
