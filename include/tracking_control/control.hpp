#ifndef TRACKING_CONTROL_CONTROL_HPP_
#define TRACKING_CONTROL_CONTROL_HPP_

#include <algorithm>
#include <iostream>
#include <limits>

#include "Eigen/Dense"
#include "tracking_control/math.hpp"

#define ENSURES(cond) \
  if (!(cond)) {      \
    std::terminate(); \
  }

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
  const auto max_lateral_thrust =
      min(sqrt(pow<2>(max_z_thrust) - pow<2>(z_sp)), lat_acc_at_full_tilt);

  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  const Eigen::Ref<const Vector2> lateral_thrust{thrust_sp.template head<2>()};

  Vector3 shaped_thrust_sp;
  shaped_thrust_sp.z() = z_sp;

  // Scale back desired lateral thrusteration if it exceeds allowed maximum
  const auto lateral_thrust_sqnorm = lateral_thrust.squaredNorm();
  if (lateral_thrust_sqnorm > pow<2>(max_lateral_thrust)) {
    const auto lateral_thrust_scale =
        max_lateral_thrust / sqrt(lateral_thrust_sqnorm);
    shaped_thrust_sp.template head<2>() = lateral_thrust_scale * lateral_thrust;
  } else {
    shaped_thrust_sp.template head<2>() = lateral_thrust;
  }

  return shaped_thrust_sp;
}

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, Derived::SizeAtCompileTime, 1> SaturationSmoothing(
    const Eigen::MatrixBase<Derived>& x, Scalar c) {
  using std::sqrt;
  if (c > Scalar(0)) {
    return x / sqrt(c + x.squaredNorm());
  }
  return x;
}

template <typename ODerived, typename WDerived,
          typename Scalar = typename ODerived::Scalar>
Eigen::Matrix<Scalar, 3, 1> BodyRatesToEulerRate(
    const Eigen::QuaternionBase<ODerived>& att,
    const Eigen::MatrixBase<WDerived>& ang_vel_rads) {
  using std::abs;
  using std::cos;
  using std::sin;
  const Eigen::Matrix angles = fsc::QuaternionToEulerAngles(att);
  const auto phi = angles.x();
  const auto theta = angles.y();

  const auto sin_theta = sin(theta);
  const auto cos_theta = cos(theta);
  const auto sin_phi = sin(phi);
  const auto cos_phi = cos(phi);
  const auto tan_theta = sin_theta / cos_theta;

  // When the vehicle pitches all the way up or all the way down, the euler
  // angles become discontinuous. In this case, we just return false.
  constexpr Scalar kIsZeroTol(1e-8);
  if (abs(cos_theta) < kIsZeroTol) {
    return Eigen::Matrix<Scalar, 3, 1>::Constant(NAN);
  }

  return {ang_vel_rads.x() + sin_phi * tan_theta * ang_vel_rads.y() +
              cos_phi * tan_theta * ang_vel_rads.z(),
          cos_phi * ang_vel_rads.y() - sin_phi * ang_vel_rads.z(),
          (sin_phi / cos_theta) * ang_vel_rads.y() +
              (cos_phi / cos_theta) * ang_vel_rads.z()};
}

// Functions in the APM namespace is reverse-engineered from the Ardupilot
// project
namespace apm {

template <typename T>
T ComputeCorrectionRate(T error, T second_ord_lim) {
  using std::abs;
  using std::copysign;
  using std::sqrt;

  const T abs_error = abs(error);
  return copysign(sqrt(T{2} * second_ord_lim * abs_error), error);
}

template <typename T>
T ComputeCorrectionRate(T error, T p, T second_ord_lim) {
  using std::abs;
  using std::copysign;
  using std::sqrt;
  const T p_sq = p * p;
  const T linear_dist = second_ord_lim / p_sq;
  const T abs_error = abs(error);
  if (abs_error > linear_dist) {
    return copysign(
        sqrt(T{2} * second_ord_lim * (abs_error - (linear_dist / T{2}))),
        error);
  }
  return error * p;
}

template <typename T>
T Proportional(T error, T p, T dt) {
  ENSURES(dt > T{0});
  ENSURES(p > T{0});
  const T correction_rate = error * p;
  return std::clamp(correction_rate, -abs(error) / dt, abs(error) / dt);
}

template <typename T>
T PiecewiseProportionalSqrt(T error, T p, T second_ord_lim, T dt) {
  ENSURES(dt > T{0});
  ENSURES(p >= T{0} && second_ord_lim > T{0});
  T correction_rate;
  using std::abs;
  using std::copysign;
  using std::fpclassify;
  using std::pow;

  if (p == T{0}) {
    // No P-control regime at all
    correction_rate = ComputeCorrectionRate(error, second_ord_lim);
  } else {
    // Both the P and second order limit have been defined.
    correction_rate = ComputeCorrectionRate(error, p, second_ord_lim);
  }
  // this ensures we do not get small oscillations by over shooting the error
  // correction in the last time step.
  return std::clamp(correction_rate, -abs(error) / dt, abs(error) / dt);
}

template <typename T>
T InvPiecewiseProportionalSqrt(T output, T p, T D_max) {
  using std::abs;
  if (D_max > T{0} && IsClose(p, T{0})) {
    return (output * output) / (2.0 * D_max);
  }
  if ((D_max < T{0} || IsClose(D_max, T{0})) && !IsClose(p, T{0})) {
    return output / p;
  }
  if ((D_max < T{0} || IsClose(D_max, T{0})) && IsClose(p, T{0})) {
    return 0.0;
  }

  // calculate the velocity at which we switch from calculating the stopping
  // point using a linear function to a sqrt function.
  const T linear_velocity = D_max / p;

  if (abs(output) < linear_velocity) {
    // if our current velocity is below the cross-over point we use a linear
    // function
    return output / p;
  }

  const T linear_dist = D_max / pow<2>(p);
  const T stopping_dist =
      (linear_dist * T{0.5}) + pow<2>(output) / (T{2.0} * D_max);
  return output > T{0} ? stopping_dist : -stopping_dist;
}

// Shapes the velocity request based on a rate time constant. The angular
// acceleration and deceleration is limited.
template <typename T>
T InputShapingBodyRates(T target_ang_vel, T desired_ang_vel, T accel_max, T dt,
                        T input_tc) {
  if (input_tc > T{0}) {
    // Calculate the acceleration to smoothly achieve rate. Jerk is not limited.
    const T error_rate = desired_ang_vel - target_ang_vel;
    const T desired_ang_accel = Proportional(error_rate, T{1} / input_tc, dt);
    desired_ang_vel = target_ang_vel + desired_ang_accel * dt;
  }
  // Acceleration is limited directly to smooth the beginning of the curve.
  if (accel_max > T{0}) {
    const T delta_ang_vel = accel_max * dt;
    return std::clamp(desired_ang_vel, target_ang_vel - delta_ang_vel,
                      target_ang_vel + delta_ang_vel);
  }
  return desired_ang_vel;
}

template <typename T>
T InputShapingAngle(T error_angle, T input_tc, T accel_max, T target_ang_vel,
                    T desired_ang_vel, T max_ang_vel, T dt) {
  ENSURES(input_tc > T{0});
  // Calculate the velocity as error approaches zero with acceleration limited
  // by accel_max_radss
  desired_ang_vel +=
      PiecewiseProportionalSqrt(error_angle, T{1} / input_tc, accel_max, dt);
  if (max_ang_vel > T{0}) {
    desired_ang_vel = std::clamp(desired_ang_vel, -max_ang_vel, max_ang_vel);
  }

  // Acceleration is limited directly to smooth the beginning of the curve.
  return InputShapingBodyRates(target_ang_vel, desired_ang_vel, accel_max, dt,
                               T{0});
}

template <typename T>
T InputShapingAngle(T error_angle, T input_tc, T accel_max, T target_ang_vel,
                    T dt) {
  ENSURES(input_tc > T{0});
  // Calculate the velocity as error approaches zero with acceleration limited
  // by accel_max_radss
  const T desired_ang_vel =
      PiecewiseProportionalSqrt(error_angle, T{1} / input_tc, accel_max, dt);

  // Acceleration is limited directly to smooth the beginning of the curve.
  return InputShapingBodyRates(target_ang_vel, desired_ang_vel, accel_max, dt,
                               T{0});
}

template <typename T>
struct ThrustCorrectionResults {
  Eigen::Quaternion<T> thrust_correction;
  Eigen::Matrix<T, 3, 1> attitude_error;
  T thrust_error_angle;
};

template <typename Derived1, typename Derived2,
          typename Scalar = typename Derived1::Scalar>
[[nodiscard]] ThrustCorrectionResults<Scalar> ThrustVectorRotationAngles(
    const Eigen::QuaternionBase<Derived1>& orientation_target,
    const Eigen::QuaternionBase<Derived2>& orientation_body) {
  using Quaternion = Eigen::Quaternion<Scalar>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using std::acos;
  using std::asin;
  using std::sin;
  using std::sqrt;

  // rotation of the body frame relative to the target body frame
  const Quaternion attitude_rel =
      orientation_body.inverse() * orientation_target;

  // Desired thrusting direction (Z-axis) of target desired body frame expressed
  // in the body frame
  const Vector3 target_thrusting_direction = attitude_rel * Vector3::UnitZ();

  // Angle between current thrusting direction and the desired thrusting
  // direction.
  // This IS acos(target_thrusting_direction.dot([0, 0, 1]))
  const auto cos_thrust_error_angle = target_thrusting_direction.z();

  Scalar thrust_error_angle;
  Quaternion thrust_correction;
  Quaternion heading_correction;
  Vector3 attitude_error;
  if (IsClose(cos_thrust_error_angle, Scalar(1))) {
    // target_thrusting_direction is almost aligned with current body z-axis
    // no thrust_correction needed
    thrust_error_angle = Scalar{0};
    attitude_error.template head<2>().setZero();
    thrust_correction.setIdentity();

    // yaw is the sole source of difference in attitude
    heading_correction = attitude_rel;
  } else {
    Vector3 rotation_axis;
    if (IsClose(cos_thrust_error_angle, Scalar(-1))) {
      // target_thrusting_direction is almost opposite to current body z-axis
      thrust_error_angle = pi_v<Scalar>;
      rotation_axis = Vector3::UnitZ();
      heading_correction = attitude_rel;
    } else {
      thrust_error_angle = acos(cos_thrust_error_angle);
      const auto normalizer = sin(thrust_error_angle);
      // A rotational axis perpendicular to both the current and target
      // thrusting direction. This IS target_thrusting_direction.cross([0, 0,
      // 1]) normalized
      rotation_axis << -target_thrusting_direction.y() / normalizer,
          target_thrusting_direction.x() / normalizer, Scalar{0};
    }
    attitude_error.template head<2>() =
        thrust_error_angle * rotation_axis.template head<2>();
    // Construct a quaternion that will just tilt the body frame to achieve the
    // desired thrusting direction
    thrust_correction = Eigen::AngleAxis{thrust_error_angle, rotation_axis};
    // Compute the remaining rotation required to achieve given yaw, after the
    // desired thrusting direction has been achieved
    heading_correction = thrust_correction.inverse() * attitude_rel;
  }

  attitude_error.z() = Scalar{2} * asin(heading_correction.z());
  return {thrust_correction, attitude_error, thrust_error_angle};
}

template <typename Derived1, typename Derived2,
          typename Scalar = typename Derived1::Scalar>
ThrustCorrectionResults<Scalar> ThrustHeadingRotationAngles(
    Eigen::QuaternionBase<Derived1>& attitude_target,
    const Eigen::QuaternionBase<Derived2>& attitude_body, Scalar kp_yaw,
    Scalar kp_yaw_rate, Scalar max_yaw_accel) {
  using std::abs;
  using std::min;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  // Thrust angle error above which yaw corrections are limited
  constexpr auto kMaxErrorAngle = deg2rad(Scalar{45});
  // minimum body-frame acceleration limit for the stability controller (for yaw
  // axis)
  constexpr auto kMinYawAccel = deg2rad(Scalar{10});  // rad/s^2
  // maximum body-frame acceleration limit for the stability controller (for yaw
  // axis)
  constexpr auto kMaxYawAccel = deg2rad(Scalar{120});  // rad/s^2

  auto res = ThrustVectorRotationAngles(attitude_target, attitude_body);

  if (IsClose(kp_yaw_rate, Scalar{0})) {
    return res;
  }
  // Limit Yaw Error to the maximum that would saturate the output when yaw rate
  // is zero.
  const Scalar heading_accel_max =
      std::clamp(max_yaw_accel / Scalar{2}, kMinYawAccel, kMaxYawAccel);
  const Scalar heading_error_max =
      min(apm::InvPiecewiseProportionalSqrt(Scalar{1} / kp_yaw_rate, kp_yaw,
                                            heading_accel_max),
          kMaxErrorAngle);
  if (IsClose(kp_yaw, Scalar{0}) ||
      abs(res.attitude_error.z()) <= heading_error_max) {
    return res;
  }
  res.attitude_error.z() = std::clamp(wrapTo2Pi(res.attitude_error.z()),
                                      -heading_error_max, heading_error_max);
  attitude_target = attitude_body * res.thrust_correction *
                    Eigen::AngleAxis{res.attitude_error.z(), Vector3::UnitZ()};
  return res;
}

template <typename Derived1, typename Derived2,
          typename Scalar = typename Derived1::Scalar>
Eigen::Matrix<Scalar, 3, 1> BodyRateLimiting(
    const Eigen::MatrixBase<Derived1>& ang_vel,
    const Eigen::MatrixBase<Derived2>& ang_vel_max) {
  using std::abs;
  using std::sqrt;

  const bool has_x_limit = ang_vel_max.x() > Scalar{0};
  const bool has_y_limit = ang_vel_max.y() > Scalar{0};
  const bool has_z_limit = ang_vel_max.z() > Scalar{0};
  Eigen::Matrix<Scalar, 3, 1> res = ang_vel;
  if (!has_x_limit || !has_y_limit) {
    // Either x- or y-component of angular velocity is limited
    // So simply clamp to limits
    if (has_x_limit) {
      res.x() = std::clamp(res.x(), -ang_vel_max.x(), ang_vel_max.x());
    }
    if (has_y_limit) {
      res.y() = std::clamp(res.y(), -ang_vel_max.y(), ang_vel_max.y());
    }
  } else {
    // Both x and y components of angular velocity are limited
    // Scale both components back evenly
    const Scalar over_limit_factor_sq =
        res.template head<2>()
            .cwiseQuotient(ang_vel_max.template head<2>())
            .squaredNorm();

    if (over_limit_factor_sq > Scalar{1}) {
      res.template head<2>() /= sqrt(over_limit_factor_sq);
    }
  }

  // Simply clamp z-component of angular velocity to limits
  if (has_z_limit) {
    res.z() = std::clamp(res.z(), -ang_vel_max.z(), ang_vel_max.z());
  }
  return res;
}
}  // namespace apm
}  // namespace fsc

#endif  // TRACKING_CONTROL_CONTROL_HPP_
