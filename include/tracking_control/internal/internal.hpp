#ifndef TRACKING_CONTROL_INTERNAL_INTERNAL_HPP_
#define TRACKING_CONTROL_INTERNAL_INTERNAL_HPP_

#include "Eigen/Dense"

namespace fsc {

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, 3, 3> accelerationVectorToRotation(
    const Eigen::MatrixBase<Derived>& acc_sp, Scalar yaw) {
  using std::cos;
  using std::sin;
  using VectorType = Eigen::Matrix<Scalar, 3, 1>;

  const auto proj_xb_des = VectorType{cos(yaw), sin(yaw), Scalar(0)};
  const VectorType zb_des = acc_sp.squaredNorm() > Scalar(0)
                                ? acc_sp.normalized()
                                : VectorType::UnitZ();
  const VectorType yb_des = zb_des.cross(proj_xb_des).normalized();
  const VectorType xb_des = yb_des.cross(zb_des).normalized();
  Eigen::Matrix<Scalar, 3, 3> rotmat;
  rotmat << xb_des, yb_des, zb_des;
  return rotmat;
}

template <typename Derived, typename Scalar = typename Derived::Scalar>
Scalar getYawFromVelocity(const Eigen::MatrixBase<Derived>& velocity) {
  using std::atan2;
  return atan2(velocity.y(), velocity.x());
}

template <typename Scalar>
constexpr Scalar sq(const Scalar& val) {
  return val * val;
}

template <typename Derived>
auto vee(const Eigen::MatrixBase<Derived>& mat) {
  using Vector3Type = Eigen::Matrix<typename Derived::Scalar, 3, 1>;
  return Vector3Type(mat(2, 1), mat(0, 2), mat(1, 0));
}

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, 3, 1> QuaternionToEulerAngles(
    const Eigen::QuaternionBase<Derived>& q) {
  using std::atan2;
  using std::sqrt;
  Eigen::Matrix<Scalar, 3, 1> angles;
  // roll (x-axis rotation)
  Scalar sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  Scalar cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles.x() = atan2(sinr_cosp, cosr_cosp);

  constexpr Scalar kPi(3.1415926535897);

  // pitch (y-axis rotation)
  Scalar sinp = sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
  Scalar cosp = sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
  angles.y() = 2 * atan2(sinp, cosp) - kPi / 2;

  // yaw (z-axis rotation)
  Scalar siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  Scalar cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles.z() = atan2(siny_cosp, cosy_cosp);

  return angles;
}
template <typename T>
inline constexpr T kDegPerRad(57.29577951308232);
template <typename T>
inline constexpr T kRadPerDeg(0.017453292519943295);

template <typename T>
constexpr T RadToDeg(T rad) {
  return rad * kDegPerRad<T>;
}

template <typename T>
constexpr T DegToRad(T deg) {
  return deg * kRadPerDeg<T>;
}
}  // namespace fsc

#endif  // TRACKING_CONTROL_INTERNAL_INTERNAL_HPP_
