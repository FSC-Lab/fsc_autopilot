#include "fsc_autopilot/attitude_control/px4_attitude_controller.hpp"

#include <iostream>
#include <limits>

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
  using std::acos;
  using std::asin;
  using std::cos;
  using std::sin;
  const auto& q = state.pose.orientation;
  Eigen::Quaterniond qd = refs.state.pose.orientation;
  const auto yawspeed_setpoint = refs.yaw_rate;

  // calculate reduced desired attitude neglecting vehicle's yaw to
  // prioritize roll and pitch
  const Eigen::Vector3d e_z = q.toRotationMatrix().col(2);
  const Eigen::Vector3d e_z_d = qd.toRotationMatrix().col(2);
  Eigen::Quaterniond qd_red = details::FromTwoVectors(e_z, e_z_d);

  if (IsClose(qd_red.x(), 1.0) || IsClose(qd_red.y(), 1.0)) {
    // In the infinitesimal corner case where the vehicle and thrust have the
    // completely opposite direction, full attitude control anyways generates
    // no yaw input and directly takes the combination of roll and pitch
    // leading to the correct desired yaw. Ignoring this case would still be
    // totally safe and stable.
    qd_red = qd;

  } else {
    // transform rotation from current to desired thrust vector into a world
    // frame reduced desired attitude
    qd_red *= q;
  }

  // mix full and reduced desired attitude
  Eigen::Quaterniond q_mix = details::Canonical(qd_red.inverse() * qd);
  // q_mix.canonicalize();
  // catch numerical problems with the domain of acosf and asinf
  q_mix.w() = std::clamp(q_mix.w(), -1.0, 1.0);
  q_mix.z() = std::clamp(q_mix.z(), -1.0, 1.0);
  qd = qd_red * Eigen::Quaterniond(cos(params_->yaw_weight * acos(q_mix.w())),
                                   0, 0,
                                   sin(params_->yaw_weight * asin(q_mix.z())));

  // quaternion attitude control law, qe is rotation from q to qd
  const Eigen::Quaterniond qe = q.inverse() * qd;

  // using sin(alpha/2) scaled rotation axis as attitude error (see
  // quaternion definition by axis angle) also taking care of the antipodal
  // unit quaternion ambiguity
  const Eigen::Vector3d eq = 2.0 * details::Canonical(qe).vec();

  // calculate angular rates setpoint
  Eigen::Vector3d rate_setpoint = eq.cwiseProduct(params_->kp_angle);

  // Feed forward the yaw setpoint rate.  yawspeed_setpoint is the feed
  // forward commanded rotation around the world z-axis, but we need to apply
  // it in the body frame (because _rates_sp is expressed in the body frame).
  // Therefore we infer the world z-axis (expressed in the body frame) by
  // taking the last column of R.transposed (== q.inverse) and multiply it by
  // the yaw setpoint rate (yawspeed_setpoint).  This yields a vector
  // representing the commanded rotatation around the world z-axis expressed
  // in the body frame such that it can be added to the rates setpoint.
  if (std::isfinite(yawspeed_setpoint)) {
    rate_setpoint += q.inverse().toRotationMatrix().col(2) * yawspeed_setpoint;
  }

  // limit rates
  if ((params_->ang_vel_max.array() > 0.0).all()) {
    rate_setpoint = rate_setpoint.cwiseMax(-params_->ang_vel_max)
                        .cwiseMin(params_->ang_vel_max);
  }

  return {Setpoint{{}, VehicleInput{0.0, rate_setpoint}},
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
