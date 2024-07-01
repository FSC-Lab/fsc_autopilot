#ifndef TRACKING_CONTROL_MATH_HPP_
#define TRACKING_CONTROL_MATH_HPP_

#include <algorithm>
#include <limits>
#include <type_traits>

#include "Eigen/Dense"

namespace fsc {

// NOLINTBEGIN(readability-identifier-naming)
template <typename T>
inline constexpr T pi_v{3.141592653589793};

inline constexpr double pi = pi_v<double>;
// NOLINTEND(readability-identifier-naming)

template <typename T>
inline constexpr T kDegPerRad(57.29577951308232);
template <typename T>
inline constexpr T kRadPerDeg(0.017453292519943295);

template <typename T>
constexpr T rad2deg(T rad) {
  static_assert(std::is_floating_point_v<T>, "Angles must be floating point");
  return rad * kDegPerRad<T>;
}

template <typename T>
constexpr T deg2rad(T deg) {
  static_assert(std::is_floating_point_v<T>, "Angles must be floating point");
  return deg * kRadPerDeg<T>;
}

template <typename T>
constexpr T wrapTo2Pi(T angle) {
  using std::fmod;
  T res = fmod(angle, T{2} * pi_v<T>);
  return res < T{0} ? res + T{2} * pi_v<T> : res;
}
template <typename T>
constexpr T wrapToPi(T angle) {
  using std::fmod;
  const T result = fmod(angle + pi_v<T>, 2.0 * pi_v<T>);
  return result <= 0.0 ? result + pi_v<T> : result - pi_v<T>;
}

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
struct Tolerances {
  static constexpr T kDefaultRelativeTol{1e-5};
  static constexpr T kDefaultAbsoluteTol{1e-8};
  T relative{kDefaultRelativeTol};
  T absolute{kDefaultAbsoluteTol};
};

template <typename T>
constexpr bool IsClose(const T& a, const T& b,
                       const Tolerances<T>& tols = Tolerances<T>()) {
  static_assert(std::is_floating_point_v<T>, "EXPECTED_FLOATING_POINT_NUMBERS");
  using std::abs;
  using std::max;
  using std::min;
  if (a == b) {
    return true;
  }

  const T diff = abs(a - b);
  const T norm = min(abs(a) + abs(b), std::numeric_limits<T>::max());
  return diff < max(tols.absolute, tols.relative * norm);
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> AngleAxisToQuaternion(
    const Eigen::MatrixBase<Derived>& angle_axis) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  using Scalar = typename Derived::Scalar;
  using std::cos;
  using std::sin;
  using std::sqrt;

  const Scalar theta_sq = angle_axis.squaredNorm();

  Scalar imag_factor;
  Scalar real_factor;
  if (IsClose(theta_sq, Scalar(0))) {
    const Scalar theta_po4 = theta_sq * theta_sq;
    // NOLINTBEGIN(readability-magic-numbers)
    imag_factor = Scalar(0.5) - Scalar(1.0 / 48.0) * theta_sq +
                  Scalar(1.0 / 3840.0) * theta_po4;
    real_factor = Scalar(1) - Scalar(1.0 / 8.0) * theta_sq +
                  Scalar(1.0 / 384.0) * theta_po4;

    // NOLINTEND(readability-magic-numbers)
  } else {
    const Scalar theta = sqrt(theta_sq);
    const Scalar half_theta = Scalar(0.5) * theta;
    const Scalar sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta / theta;
    real_factor = cos(half_theta);
  }

  Eigen::Quaternion<Scalar> quaternion;
  quaternion.w() = real_factor;
  quaternion.vec() = imag_factor * angle_axis;
  return quaternion;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> QuaternionToAngleAxis(
    const Eigen::QuaternionBase<Derived>& quaternion) {
  using Scalar = typename Derived::Scalar;

  using std::atan2;
  using std::sqrt;
  const Scalar squared_n = quaternion.vec().squaredNorm();
  const Scalar& w = quaternion.w();

  Scalar two_atan_nbyw_by_n;

  if (IsClose(squared_n, Scalar(0))) {
    // If quaternion is normalized and n=0, then w should be 1;
    // w=0 should never happen here!
    const Scalar squared_w = w * w;

    // NOLINTBEGIN(readability-magic-numbers)
    two_atan_nbyw_by_n =
        Scalar(2.0) / w - Scalar(2.0 / 3.0) * (squared_n) / (w * squared_w);
    // NOLINTEND(readability-magic-numbers)
  } else {
    const Scalar n = sqrt(squared_n);

    // w < 0 ==> cos(theta/2) < 0 ==> theta > pi
    //
    // By convention, the condition |theta| < pi is imposed by wrapping theta
    // to pi; The wrap operation can be folded inside evaluation of atan2
    //
    // theta - pi = atan(sin(theta - pi), cos(theta - pi))
    //            = atan(-sin(theta), -cos(theta))
    //
    const Scalar atan_nbyw = (w < Scalar(0)) ? atan2(-n, -w) : atan2(n, w);
    two_atan_nbyw_by_n = Scalar(2) * atan_nbyw / n;
  }

  return two_atan_nbyw_by_n * quaternion.vec();
}

}  // namespace fsc

#endif  // TRACKING_CONTROL_MATH_HPP_
