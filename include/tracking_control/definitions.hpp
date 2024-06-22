#ifndef TRACKING_CONTROL_DEFINITIONS_HPP_
#define TRACKING_CONTROL_DEFINITIONS_HPP_
#include <memory>

#include "Eigen/Core"      // IWYU pragma: keep
#include "Eigen/Geometry"  // IWYU pragma: keep

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
}  // namespace fsc

#endif  // TRACKING_CONTROL_DEFINITIONS_HPP_
