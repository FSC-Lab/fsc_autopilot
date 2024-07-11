#ifndef TRACKING_CONTROL_DEFINITIONS_HPP_
#define TRACKING_CONTROL_DEFINITIONS_HPP_
#include <string>

#include "Eigen/Core"      // IWYU pragma: keep
#include "Eigen/Geometry"  // IWYU pragma: keep
#include "tracking_control/vehicle_input.hpp"

namespace fsc {
struct Pose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct Twist {
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct Accel {
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct VehicleState {
  double stamp;
  Pose pose;
  Twist twist;
  Accel accel;
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
  double yaw_rate;
  ReferenceKind kind;
};

struct Setpoint {
  VehicleState state;
  VehicleInput input;
};

struct ContextBase {
  virtual ~ContextBase() = default;

  [[nodiscard]] virtual std::string message() const { return ""; }

  [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace fsc

#endif  // TRACKING_CONTROL_DEFINITIONS_HPP_
