#ifndef TRACKING_CONTROL_MULTIROTOR_ACCELERATION_LIMITER_HPP_
#define TRACKING_CONTROL_MULTIROTOR_ACCELERATION_LIMITER_HPP_

#include <algorithm>
#include <limits>

#include "Eigen/Dense"
#include "tracking_control/internal/internal.hpp"

namespace fsc {

template <typename Scalar>
struct AccelerationBounds {
  Scalar lower{0};
  Scalar upper{std::numeric_limits<Scalar>::max()};
};

namespace details {
template <int IntPow, typename Scalar>
constexpr Scalar pow(Scalar value) {
  static_assert(IntPow >= 0, "Negative powers are not supported");

  if constexpr (IntPow == 0) {
    return Scalar(1);
  } else if constexpr (IntPow == 1) {
    return value;
  } else {
    return pow<IntPow - 1>(value) * value;
  }
}

}  // namespace details

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, 3, 1> MultirotorAccelerationLimiting(
    const Eigen::MatrixBase<Derived>& accel_sp,
    const AccelerationBounds<Scalar>& accel_bnds, Scalar max_tilt_angle) {
  using details::pow;
  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;
  using std::tan;

  auto [min_z_accel, max_z_accel] = accel_bnds;
  if (accel_sp.z() > max_z_accel) {
    // Lift alone saturates actuators, deliver as much lift as possible and no
    // lateral acceleration
    return {Scalar(0), Scalar(0), max_z_accel};
  }

  // Lift does not saturate actuators, deliver a minimum of min_z_accel
  const auto z_sp = max(accel_sp.z(), min_z_accel);
  // If tilt limiting is inactive, allowed lateral acceleration is infinity
  const auto lat_acc_at_full_tilt =
      max_tilt_angle > Scalar(0)
          ? abs(z_sp) * tan(fsc::DegToRad(max_tilt_angle))
          : std::numeric_limits<Scalar>::max();

  // Aim to deliver requested lift exactly while scaling
  // back maximally allowed lateral acceleration subject
  // to either acceleration limits or tilt limits
  const auto max_lateral_acc =
      min(sqrt(pow<2>(max_z_accel) - pow<2>(z_sp)), lat_acc_at_full_tilt);

  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  const Eigen::Ref<const Vector2> lateral_acc{accel_sp.template head<2>()};

  Vector3 shaped_accel_sp;
  shaped_accel_sp.z() = z_sp;

  // Scale back desired lateral acceleration if it exceeds allowed maximum
  const auto lateral_acc_sqnorm = lateral_acc.squaredNorm();
  if (lateral_acc_sqnorm > pow<2>(max_lateral_acc)) {
    const auto lateral_acc_scale = max_lateral_acc / sqrt(lateral_acc_sqnorm);
    shaped_accel_sp.template head<2>() = lateral_acc_scale * lateral_acc;
  } else {
    shaped_accel_sp.template head<2>() = lateral_acc;
  }

  return shaped_accel_sp;
}
}  // namespace fsc
#endif  // TRACKING_CONTROL_MULTIROTOR_ACCELERATION_LIMITER_HPP_
