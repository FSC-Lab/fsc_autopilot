#ifndef TRACKING_CONTROL_INTERNAL_INTERNAL_HPP_
#define TRACKING_CONTROL_INTERNAL_INTERNAL_HPP_

#include "Eigen/Dense"

namespace details {

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

template <typename Scalar>
constexpr Scalar deg2rad(const Scalar& val) {
  constexpr auto kRad2DegFactor = 0.017453292519943295L;
  return val * Scalar(kRad2DegFactor);
}

template <typename Derived>
auto vee(const Eigen::MatrixBase<Derived>& mat) {
  using Vector3Type = Eigen::Matrix<typename Derived::Scalar, 3, 1>;
  return Vector3Type(mat(2, 1), mat(0, 2), mat(1, 0));
}
}  // namespace details

#endif  // TRACKING_CONTROL_INTERNAL_INTERNAL_HPP_
