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

class Context {
 public:
  virtual ~Context() = default;

  virtual bool get([[maybe_unused]] std::string_view name,
                   [[maybe_unused]] double& value) const {
    return false;
  }

  virtual bool set([[maybe_unused]] std::string_view name,
                   [[maybe_unused]] double value) {
    return false;
  }

  [[nodiscard]] virtual bool get([[maybe_unused]] std::string_view name,
                                 [[maybe_unused]] bool& value) const {
    return false;
  }

  virtual bool set([[maybe_unused]] std::string_view name,
                   [[maybe_unused]] bool value) {
    return false;
  }

  [[nodiscard]] virtual bool get(
      [[maybe_unused]] std::string_view name,
      [[maybe_unused]] Eigen::Ref<Eigen::VectorXd>& x) const {
    return false;
  }

  [[nodiscard]] bool set(
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
}  // namespace fsc

#endif  // TRACKING_CONTROL_DEFINITIONS_HPP_
