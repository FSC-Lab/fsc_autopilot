// Copyright © 2024 FSC Lab
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_CONTROL_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_CONTROL_HPP_

#include <algorithm>

#include "Eigen/Dense"
#include "fsc_autopilot/math/math_extras.hpp"
#include "fsc_autopilot/math/numbers.hpp"
#include "fsc_autopilot/math/rotation.hpp"

#define ENSURES(cond) \
  if (!(cond)) {      \
    std::terminate(); \
  }

#define FWD(...) std::forward<decltype(__VA_ARGS__)>(__VA_ARGS__)

namespace fsc {

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
        sqrt(T{2} * second_ord_lim * (abs_error - linear_dist / T{2})), error);
  }
  return error * p;
}

template <typename T>
T Proportional(T error, T p, T dt) {
  ENSURES(dt > T{0});
  ENSURES(p > T{0});
  using std::abs;

  const T bound = abs(error) / dt;
  return std::clamp(p * error, -bound, bound);
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
  using std::copysign;
  if (D_max > T{0} && IsClose(p, T{0})) {
    return (output * output) / (T{2} * D_max);
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
      linear_dist / T{2} + pow<2>(output) / (T{2.0} * D_max);
  return copysign(stopping_dist, output);
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
    const Eigen::QuaternionBase<Derived1>& attitude_target,
    const Eigen::QuaternionBase<Derived2>& attitude_body) {
  using Quaternion = Eigen::Quaternion<Scalar>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using std::acos;
  using std::asin;
  using std::atan2;
  using std::sqrt;

  const Quaternion att_target_to_body =
      attitude_body.inverse() * attitude_target;

  const Scalar qe_x = att_target_to_body.x();
  const Scalar qe_y = att_target_to_body.y();
  const Scalar qe_z = att_target_to_body.z();
  const Scalar qe_w = att_target_to_body.w();
  const Scalar qt_w_sq = pow<2>(qe_w) + pow<2>(qe_z);
  Scalar heading_error_angle;

  Quaternion thrust_vector_correction;
  Scalar thrust_error_angle;

  if (IsClose(qt_w_sq, Scalar{0})) {
    // When w_sq = cos(thrust_error_angle / 2) == 0, thrust_error_angle = 180
    // degrees, aka thrust_vec_body and thrust_vec_target are directly opposite.
    // Just rotating by att_target_to_body corrects the thrust
    thrust_error_angle = numbers::pi_v<Scalar>;
    thrust_vector_correction = {Scalar{0}, qe_x, qe_y, Scalar{0}};
    heading_error_angle = Scalar{0};
  } else {
    const Scalar qt_w = sqrt(qt_w_sq);
    const Scalar i_qt_w = Scalar{1} / qt_w;
    const Scalar qe_w_by_qt_w = i_qt_w * qe_w;
    const Scalar qe_z_by_qt_w = i_qt_w * qe_z;

    thrust_vector_correction = {qt_w, qe_w_by_qt_w * qe_x - qe_z_by_qt_w * qe_y,
                                qe_w_by_qt_w * qe_y + qe_z_by_qt_w * qe_x,
                                Scalar{0}};
    heading_error_angle = Scalar{2} * (qe_w < Scalar{0} ? atan2(-qe_z, -qe_w)
                                                        : atan2(qe_z, qe_w));
    thrust_error_angle = acos(Scalar{2} * qt_w_sq - Scalar{1});
  }

  // calculate the angle error in x and y.
  Vector3 attitude_error = fsc::QuaternionToAngleAxis(thrust_vector_correction);
  attitude_error.z() = heading_error_angle;

  return {thrust_vector_correction, attitude_error, thrust_error_angle};
}

template <typename T>
T YawErrorLimiting(T yaw_error, T kp_yaw, T kp_yaw_rate, T max_yaw_accel) {
  using std::abs;
  using std::min;

  if (IsClose(kp_yaw_rate, T{0})) {
    return yaw_error;
  }

  if (IsClose(kp_yaw, T{0})) {
    return yaw_error;
  }

  constexpr auto kMinYawAccel = deg2rad(T{10});   // rad/s^2
  constexpr auto kMaxYawAccel = deg2rad(T{120});  // rad/s^2

  // Limit Yaw Error to the maximum that would saturate the output when yaw
  // rate is zero.
  const T heading_accel_max =
      std::clamp(max_yaw_accel / T{2}, kMinYawAccel, kMaxYawAccel);

  // Thrust angle error above which yaw corrections are limited
  constexpr auto kMaxErrorAngle = deg2rad(T{45});
  const T heading_error_max =
      min(apm::InvPiecewiseProportionalSqrt(T{1} / kp_yaw_rate, kp_yaw,
                                            heading_accel_max),
          kMaxErrorAngle);

  return std::clamp(wrapToPi(yaw_error), -heading_error_max, heading_error_max);
}

template <typename Derived1, typename Derived2,
          typename Scalar = typename Derived1::Scalar>
ThrustCorrectionResults<Scalar> ThrustHeadingRotationAngles(
    Eigen::QuaternionBase<Derived1>& attitude_target,
    const Eigen::QuaternionBase<Derived2>& attitude_body, Scalar kp_yaw,
    Scalar kp_yaw_rate, Scalar max_yaw_accel) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  auto res = ThrustVectorRotationAngles(attitude_target, attitude_body);

  res.attitude_error.z() = YawErrorLimiting(res.attitude_error.z(), kp_yaw,
                                            kp_yaw_rate, max_yaw_accel);

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

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_CONTROL_HPP_
