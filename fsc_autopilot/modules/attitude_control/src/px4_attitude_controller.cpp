#include "fsc_autopilot/attitude_control/px4_attitude_controller.hpp"

#include <iostream>
#include <limits>

#include "fsc_autopilot/attitude_control/control.hpp"
#include "fsc_autopilot/math/math_extras.hpp"

namespace fsc {
namespace details {
template <typename T>
int sgn(T val) {
  return (static_cast<T>(0) < val) - (val < static_cast<T>(0));
}

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Quaternion<Scalar> Canonical(const Eigen::QuaternionBase<Derived>& q) {
  using std::abs;

  constexpr auto kTolerance = std::numeric_limits<Scalar>::epsilon();
  for (Eigen::Index j = 0; j < 4; j++) {
    auto i = (j + 7) % 4;
    if (abs(q.coeffs().coeff(i)) > kTolerance) {
      return Eigen::Quaternion<Scalar>(
          q.coeffs() * static_cast<Scalar>(sgn(q.coeffs().coeff(i))));
    }
  }

  return q;
}
template <typename Derived1, typename Derived2,
          typename Scalar = typename Derived1::Scalar>
EIGEN_DEVICE_FUNC Eigen::Quaternion<Scalar> FromTwoVectors(
    const Eigen::MatrixBase<Derived1>& a,
    const Eigen::MatrixBase<Derived2>& b) {
  using std::sqrt;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  Eigen::Quaternion<Scalar> q;
#ifdef USE_EIGEN_IMPL

  Vector3 v0 = a.normalized();
  Vector3 v1 = b.normalized();
  Scalar c = v1.dot(v0);

  // if dot == -1, vectors are nearly opposites
  // => accurately compute the rotation axis by computing the
  //    intersection of the two planes. This is done by solving:
  //       x^T v0 = 0
  //       x^T v1 = 0
  //    under the constraint:
  //       ||x|| = 1
  //    which yields a singular value problem
  if (c < Scalar(-1) + std::numeric_limits<Scalar>::epsilon()) {
    return Eigen::Quaternion<Scalar>::Identity();
  }
  Vector3 axis = v0.cross(v1);
  Scalar s = sqrt((Scalar(1) + c) * Scalar(2));
  Scalar invs = Scalar(1) / s;
  q.vec() = axis * invs;
  q.w() = s * Scalar(0.5);
#else
  Vector3 cr = a.cross(b);
  const auto dt = a.dot(b);

  constexpr auto kEps = std::numeric_limits<Scalar>::epsilon();
  if (cr.norm() < kEps && dt < 0) {
    // handle corner cases with 180 degree rotations
    // if the two vectors are parallel, cross product is zero
    // if they point opposite, the dot product is negative
    cr = a.cwiseAbs();

    if (cr(0) < cr(1)) {
      if (cr(0) < cr(2)) {
        cr = Vector3(1, 0, 0);

      } else {
        cr = Vector3(0, 0, 1);
      }

    } else {
      if (cr(1) < cr(2)) {
        cr = Vector3(0, 1, 0);

      } else {
        cr = Vector3(0, 0, 1);
      }
    }

    q.w() = static_cast<Scalar>(0);
    cr = a.cross(cr);

  } else {
    // normal case, do half-way quaternion solution
    q.w() = dt + std::sqrt(a.squaredNorm() * b.squaredNorm());
  }

  q.vec() = cr;
  q.normalize();

#endif

  return q;
}
}  // namespace details

ControlResult PX4AttitudeController::run(const VehicleState& state,
                                         const Reference& refs,
                                         [[maybe_unused]] ContextBase* error) {
  using std::abs;
  using std::asin;
  using std::atan2;
  using std::sin;
  using std::sqrt;
  const auto& q = state.pose.orientation;
  Eigen::Quaterniond qd = refs.state.pose.orientation;
  const auto yawspeed_setpoint = refs.yaw_rate;

  const Eigen::Quaterniond qe = q.inverse() * qd;

  Eigen::Quaterniond qe_red;
  Eigen::Vector3d eq;
  const double w_sq = pow<2>(qe.w()) + pow<2>(qe.z());
  if (IsClose(w_sq, 0.0)) {
    eq = {qe.x(), qe.y(), 0.0};

  } else {
    const auto w = sqrt(w_sq);
    const auto iw = 1.0 / w;
    const auto qe0_by_w = iw * qe.w();
    const auto qe3_by_w = iw * qe.z();

    const auto yaw_e_angle =
        qe.w() < 0.0 ? atan2(-qe.z(), -qe.w()) : atan2(qe.z(), qe.w());
    eq = {qe0_by_w * qe.x() - qe3_by_w * qe.y(),
          qe0_by_w * qe.y() + qe3_by_w * qe.x(),
          sin(params_->yaw_weight * yaw_e_angle)};
  }

  // calculate angular rates setpoint
  Eigen::Vector3d ang_vel_target = eq.cwiseProduct(params_->kp_angle);

  // Feed forward the yaw setpoint rate.
  // yawspeed_setpoint is the feed forward commanded rotation around the world
  // z-axis, but we need to apply it in the body frame (because _rates_sp is
  // expressed in the body frame). Therefore we infer the world z-axis
  // (expressed in the body frame) by taking the last column of R.transposed (==
  // q.inversed) and multiply it by the yaw setpoint rate (yawspeed_setpoint).
  // This yields a vector representing the commanded rotatation around the world
  // z-axis expressed in the body frame such that it can be added to the rates
  // setpoint.
  if (std::isfinite(yawspeed_setpoint)) {
    ang_vel_target += q.inverse().toRotationMatrix().col(2) * yawspeed_setpoint;
  }

  // limit rates
  ang_vel_target = apm::BodyRateLimiting(ang_vel_target, params_->ang_vel_max);

  return {Setpoint{{}, VehicleInput{0.0, ang_vel_target}},
          ControllerErrc::kSuccess};
}

bool PX4AttitudeControllerParams::load(const ParameterLoaderBase& loader,
                                       LoggerBase* logger) {
  std::ignore = loader.getParam("roll_p", kp_angle.x());
  std::ignore = loader.getParam("pitch_p", kp_angle.y());
  std::ignore = loader.getParam("yaw_p", kp_angle.z());
  return true;
}

std::string PX4AttitudeControllerParams::toString() const {
  std::ostringstream oss;
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};

  oss << kp_angle.format(f) << "\n";
  return oss.str();
}
}  // namespace fsc
