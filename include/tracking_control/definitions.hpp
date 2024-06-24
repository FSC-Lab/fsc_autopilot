#ifndef TRACKING_CONTROL_DEFINITIONS_HPP_
#define TRACKING_CONTROL_DEFINITIONS_HPP_
#include <string>
#include <system_error>
#include <type_traits>

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

enum class ControllerErrc {
  kSuccess = 0,
  kInvalidState = 1,
  kInvalidReference = 2,
  kInvalidParameters = 3,
  kComputationError = 4
};

namespace detail {
class ControllerErrcCategory final : public std::error_category {
 public:
  [[nodiscard]] const char* name() const noexcept final {
    return "ControllerError";
  }

  [[nodiscard]] std::string message(int c) const final {
    switch (static_cast<fsc::ControllerErrc>(c)) {
      case fsc::ControllerErrc::kSuccess:
        return "controller successful";
      case fsc::ControllerErrc::kInvalidState:
        return "system state is invalid";
      case fsc::ControllerErrc::kInvalidReference:
        return "reference is invalid";
      case fsc::ControllerErrc::kInvalidParameters:
        return "controller parameters are invalid";
      case fsc::ControllerErrc::kComputationError:
        return "error in controller computation";
      default:
        return "Invalid error code";
    }
  }
};
}  // namespace detail

extern inline const detail::ControllerErrcCategory&
GetControllerErrcCategory() {
  static detail::ControllerErrcCategory c;
  return c;
}

inline std::error_code make_error_code(fsc::ControllerErrc errc) {
  return {static_cast<int>(errc), GetControllerErrcCategory()};
}
}  // namespace fsc

namespace std {

template <>
struct is_error_code_enum<::fsc::ControllerErrc> : true_type {};

}  // namespace std

#endif  // TRACKING_CONTROL_DEFINITIONS_HPP_
