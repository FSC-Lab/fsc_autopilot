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

#ifndef FSC_AUTOPILOT_POSITION_CONTROL_CONTROL_HPP_
#define FSC_AUTOPILOT_POSITION_CONTROL_CONTROL_HPP_

#include <limits>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/math/math_extras.hpp"
#include "fsc_autopilot/utils/asserts.hpp"

namespace fsc {

enum class PIDForm { kIdeal, kNested, kParallel };

inline std::string to_string(PIDForm form) {
  switch (form) {
    case PIDForm::kIdeal:
      return "ideal";
    case PIDForm::kNested:
      return "nested";
    case PIDForm::kParallel:
      return "parallel";
  }
  FSC_ASSUME(false);
}

template <typename T>
T ProportionalDerivative(T error, T error_dot, T kp, T kd, T dt,
                         PIDForm form = PIDForm::kIdeal) {
  using std::abs;
  T res{0};
  switch (form) {
    case PIDForm::kIdeal:
      // Kp acts on the sum of all terms, and Kd can be interpreted as some
      // inverse time constant
      res = kp * (error + kd * error_dot);
      break;
    case PIDForm::kNested:
      // "Second-order" feedback, where kp * error is on the same order as
      // error_dot, and Kd acts on all "second order" terms
      res = kd * (error_dot + kp * error);
      break;
    case PIDForm::kParallel:
      // Traditional PD with separate gains
      res = kp * error * kd * error_dot;
      break;
  }
  const T bound = abs(error) / dt;
  return std::clamp(res, -bound, bound);
}

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

  [[nodiscard]] bool valid() const { return upper > lower; }
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
      min(sqrt(pown<2>(max_z_thrust) - pown<2>(z_sp)), lat_acc_at_full_tilt);

  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  const Eigen::Ref<const Vector2> lateral_thrust{thrust_sp.template head<2>()};

  Vector3 shaped_thrust_sp;
  shaped_thrust_sp.z() = z_sp;

  // Scale back desired lateral thrusteration if it exceeds allowed maximum
  const auto lateral_thrust_sqnorm = lateral_thrust.squaredNorm();
  if (lateral_thrust_sqnorm > pown<2>(max_lateral_thrust)) {
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
}  // namespace fsc

#endif  // FSC_AUTOPILOT_POSITION_CONTROL_CONTROL_HPP_
