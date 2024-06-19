#ifndef TRACKING_CONTROL_CONTROL_HPP_
#define TRACKING_CONTROL_CONTROL_HPP_

#include <algorithm>
#include <limits>

#include "Eigen/Dense"
#include "tracking_control/math.hpp"

namespace fsc {
template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, 3, 3> thrustVectorToRotation(
    const Eigen::MatrixBase<Derived>& acc_sp, Scalar yaw) {
  using std::cos;
  using std::sin;
  using VectorType = Eigen::Matrix<Scalar, 3, 1>;

  const VectorType proj_xb_des{cos(yaw), sin(yaw), Scalar(0)};
  const VectorType zb_des = acc_sp.squaredNorm() > Scalar(0)
                                ? acc_sp.normalized()
                                : VectorType::UnitZ();
  const VectorType yb_des = zb_des.cross(proj_xb_des).normalized();
  const VectorType xb_des = yb_des.cross(zb_des).normalized();
  Eigen::Matrix<Scalar, 3, 3> rotmat;
  rotmat << xb_des, yb_des, zb_des;
  return rotmat;
}

template <typename Scalar>
struct ThrustBounds {
  Scalar lower{0};
  Scalar upper{std::numeric_limits<Scalar>::max()};
};

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, 3, 1> MultirotorThrustLimiting(
    const Eigen::MatrixBase<Derived>& thrust_sp,
    const ThrustBounds<Scalar>& thrust_bnds, Scalar max_tilt_angle) {
  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;
  using std::tan;

  auto [min_z_thrust, max_z_thrust] = thrust_bnds;
  if (thrust_sp.z() > max_z_thrust) {
    // Lift alone saturates actuators, deliver as much lift as possible and no
    // lateral thrust
    return {Scalar(0), Scalar(0), max_z_thrust};
  }

  // Lift does not saturate actuators, deliver a minimum of min_z_thrust
  const auto z_sp = max(thrust_sp.z(), min_z_thrust);
  // If tilt limiting is inactive, allowed lateral thrusteration is infinity
  const auto lat_acc_at_full_tilt =
      max_tilt_angle > Scalar(0) ? abs(z_sp) * tan(fsc::deg2rad(max_tilt_angle))
                                 : std::numeric_limits<Scalar>::max();

  // Aim to deliver requested lift exactly while scaling
  // back maximally allowed lateral thrusteration subject
  // to either thrusteration limits or tilt limits
  const auto max_lateral_acc =
      min(sqrt(pow<2>(max_z_thrust) - pow<2>(z_sp)), lat_acc_at_full_tilt);

  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  const Eigen::Ref<const Vector2> lateral_acc{thrust_sp.template head<2>()};

  Vector3 shaped_thrust_sp;
  shaped_thrust_sp.z() = z_sp;

  // Scale back desired lateral thrusteration if it exceeds allowed maximum
  const auto lateral_acc_sqnorm = lateral_acc.squaredNorm();
  if (lateral_acc_sqnorm > pow<2>(max_lateral_acc)) {
    const auto lateral_acc_scale = max_lateral_acc / sqrt(lateral_acc_sqnorm);
    shaped_thrust_sp.template head<2>() = lateral_acc_scale * lateral_acc;
  } else {
    shaped_thrust_sp.template head<2>() = lateral_acc;
  }

  return shaped_thrust_sp;
}
}  // namespace fsc

#endif  // TRACKING_CONTROL_CONTROL_HPP_
