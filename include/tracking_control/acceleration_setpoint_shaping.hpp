#ifndef TRACKING_CONTROL_ACCELERATION_SETPOINT_SHAPER_HPP_
#define TRACKING_CONTROL_ACCELERATION_SETPOINT_SHAPER_HPP_

#include <algorithm>
#include <limits>

#include "Eigen/Dense"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"

namespace control {
template <typename Derived>
class AccelerationSetpointShaping {
 public:
  using Scalar = typename ControllerTraits<Derived>::Scalar;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  const Derived& derived() const { return static_cast<const Derived&>(*this); }
  Derived& derived() { return static_cast<Derived&>(*this); }

  Scalar max_z_accel() const { return derived().max_z_accel(); }
  Scalar min_z_accel() const { return derived().min_z_accel(); }
  Scalar max_tilt_ratio() const { return derived().max_tilt_ratio(); }

  Vector3 reshapeAccelerationSetpoint(const Vector3& accel_sp) const {
    using details::sq;
    using std::abs;
    using std::max;
    using std::min;
    using std::sqrt;

    if (accel_sp.z() > max_z_accel()) {
      // Lift alone saturates actuators, deliver as much lift as possible and no
      // lateral acceleration
      return {Scalar(0), Scalar(0), max_z_accel()};
    }

    // Lift does not saturate actuators, deliver a minimum of list
    const auto z_sp = max(accel_sp.z(), min_z_accel());

    // Aim to deliver requested lift exactly while scaling back maximally
    // allowed lateral acceleration subject to either acceleration limits or
    // tilt limits
    const auto max_lateral_acc = min(sqrt(sq(max_z_accel()) - sq(z_sp)),
                                     getLateralAccelerationAtMaxTilt(z_sp));

    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    const Eigen::Ref<const Vector2> lateral_acc{accel_sp.template head<2>()};

    Vector3 shaped_accel_sp;
    shaped_accel_sp.z() = z_sp;

    // Scale back desired lateral acceleration if it exceeds allowed maximum
    const auto lateral_acc_sqnorm = lateral_acc.squaredNorm();
    if (lateral_acc_sqnorm > sq(max_lateral_acc)) {
      const auto lateral_acc_scale = max_lateral_acc / sqrt(lateral_acc_sqnorm);
      shaped_accel_sp.template head<2>() = lateral_acc_scale * lateral_acc;
    } else {
      shaped_accel_sp.template head<2>() = lateral_acc;
    }
    return shaped_accel_sp;
  }

 private:
  Scalar getLateralAccelerationAtMaxTilt(Scalar z_sp) const {
    using std::abs;
    if (max_tilt_ratio() > Scalar(0)) {
      return abs(z_sp) * max_tilt_ratio();
    }

    // Tilt limiting is inactive, so allowed lateral acceleration is infinity
    return std::numeric_limits<Scalar>::infinity();
  }
};
}  // namespace control

#endif  // TRACKING_CONTROL_ACCELERATION_SETPOINT_SHAPER_HPP_
