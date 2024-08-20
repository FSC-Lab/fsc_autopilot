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

#ifndef FSC_AUTOPILOT_MATH_ROTATION_HPP_
#define FSC_AUTOPILOT_MATH_ROTATION_HPP_

#include "Eigen/Dense"
#include "fsc_autopilot/math/math_extras.hpp"

namespace fsc {

/**
 * @brief Convert an angle-axis vector into a skew-symmetric matrix representing
 * an element in the \f$\mathfrak{so}(3)\f$ Lie algebra
 *
 * @param phi The angle-axis vector (c.f. rotation vector)
 * @return `Phi` A 3-by-3 matrix consisting of
 * \f$\left[\begin{array}{ccc} 0 & -\phi_2 & \phi_1 \\ \phi_2 & 0 & -phi_0 \\
 * -\phi_1 & \phi_0 & 0\end{array}\right] \f$
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> hat(
    const Eigen::MatrixBase<Derived>& phi);

/**
 * @brief Convert a skew-symmetric matrix representing an element in the
 * \f$\mathfrak{so}(3)\f$ Lie algebra into an angle-axis vector
 *
 * @param Phi A skew-symmetric matrix
 * @return `phi` A 3-vector consisting of
 * \f$\left[\begin{array}{ccc}\Phi_{21} & \Phi_{02} &
 * \Phi_{10}\end{array}\right] \f$
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> vee(
    const Eigen::MatrixBase<Derived>& mat);

/**
 * @brief Convert a angle-axis vector (c.f. rotation vector / Rodrigues vector)
 * into the equivalent rotation matrix.
 *
 * @param phi The angle-axis vector, whose norm is an angle in radians and
 * whose direction is aligned with the axis of rotation
 * @return `R` The equivalent rotation matrix, a 3-by-3 matrix that is
 * orthonormal with determinant 1
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> AngleAxisToRotationMatrix(
    const Eigen::MatrixBase<Derived>& angle_axis);

/**
 * @brief Convert a angle-axis vector (c.f. rotation vector / Rodrigues vector)
 * into the equivalent unit quaternion
 *
 * @param phi The angle-axis vector, whose norm is an angle in radians and
 * whose direction is aligned with the axis of rotation
 * @return `quaternion` The equivalent unit quaternion
 */
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> AngleAxisToQuaternion(
    const Eigen::MatrixBase<Derived>& angle_axis);

/**
 * @brief Convert a rotation matrix into the equivalent angle-axis vector (c.f.
 * rotation vector / Rodrigues vector)
 *
 * @param R The rotation matrix, a 3-by-3 matrix that is orthonormal
 * with determinant 1
 * @return `phi` The equivalent angle-axis vector, whose norm is an angle in
 * radians and whose direction is aligned with the axis of rotation
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> RotationMatrixToAngleAxis(
    const Eigen::MatrixBase<Derived>& rotation_matrix);

/**
 * @brief Convert a rotation matrix into the equivalent unit quaternion. This
 * function is provided for completeness, it is a simple wrapper over the
 * `Eigen::Quaternion` constructor
 *
 * @param R The rotation matrix, a 3-by-3 matrix that is orthonormal
 * with determinant 1
 * @return `quaternion` The equivalent unit quaternion
 */
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> RotationMatrixToQuaternion(
    const Eigen::MatrixBase<Derived>& rotation_matrix);

/**
 * @brief Convert an unit quaternion into the equivalent angle-axis vector (c.f.
 * rotation vector / Rodrigues vector)
 *
 * @param quaternion An unit quaternion
 * @return `phi` The equivalent angle-axis vector, whose norm is an angle in
 * radians and whose direction is aligned with the axis of rotation
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> QuaternionToAngleAxis(
    const Eigen::QuaternionBase<Derived>& quaternion);

/**
 * @brief Convert an unit quaternion into the equivalent rotation matrix
 *
 * @param quaternion An unit quaternion
 * @return `R` The equivalent rotation matrix, a 3-by-3 matrix that is
 * orthonormal with determinant 1
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> QuaternionToRotationMatrix(
    const Eigen::QuaternionBase<Derived>& quaternion);

/***************************BEGIN OF IMPLEMENTATION****************************/

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> hat(
    const Eigen::MatrixBase<Derived>& phi) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  using Scalar = typename Derived::Scalar;

  Eigen::Matrix<Scalar, 3, 3> matrix;
  matrix << Scalar(0), -phi.coeff(2), phi.coeff(1),  //
      phi.coeff(2), Scalar(0), -phi.coeff(0),        //
      -phi.coeff(1), phi.coeff(0), Scalar(0);
  return matrix;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> vee(
    const Eigen::MatrixBase<Derived>& mat) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
  return {mat.coeff(2, 1), mat.coeff(0, 2), mat.coeff(1, 0)};
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> AngleAxisToRotationMatrix(
    const Eigen::MatrixBase<Derived>& angle_axis) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  using std::cos;
  using std::sin;
  using std::sqrt;
  using Scalar = typename Derived::Scalar;

  const Scalar theta_sq = angle_axis.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> rotation_matrix =
      Eigen::Matrix<Scalar, 3, 3>::Identity();
  const Eigen::Matrix<Scalar, 3, 3> hat_phi = hat(angle_axis);
  const Eigen::Matrix<Scalar, 3, 3> hat_phi_sq = hat_phi * hat_phi;

  if (IsClose(theta_sq, Scalar(0))) {
    rotation_matrix += hat_phi + hat_phi_sq / Scalar(2);
  } else {
    const Scalar theta = sqrt(theta_sq);
    const Scalar cos_theta = cos(theta);
    const Scalar sin_theta = sin(theta);

    rotation_matrix += (Scalar(1) - cos_theta) / theta_sq * hat_phi_sq +
                       sin_theta / theta * hat_phi;
  }

  return rotation_matrix;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> RotationMatrixToAngleAxis(
    const Eigen::MatrixBase<Derived>& R) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);

  return QuaternionToAngleAxis(RotationMatrixToQuaternion(R));
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
    two_atan_nbyw_by_n =
        Scalar(2.0) / w - Scalar(2.0 / 3.0) * (squared_n) / (w * squared_w);
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

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> QuaternionToRotationMatrix(
    const Eigen::QuaternionBase<Derived>& quaternion) {
  using Scalar = typename Derived::Scalar;
  return quaternion.toRotationMatrix();
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> RotationMatrixToQuaternion(
    const Eigen::MatrixBase<Derived>& R) {
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);

  // Simply forward R to the Quaternion constructor. Even though Eigen's
  // implementation of the Shomake algorithm can be optimized, any
  // re-implementation wastes time in default-constructing the quaternion
  return Eigen::Quaternion<Scalar>(R);
}

template <typename Derived, typename Scalar = typename Derived::Scalar>
Eigen::Matrix<Scalar, 3, 1> QuaternionToEulerAngles(
    const Eigen::QuaternionBase<Derived>& q) {
  using std::atan2;
  using std::sqrt;
  Eigen::Matrix<Scalar, 3, 1> angles;
  // roll (x-axis rotation)
  Scalar sinr_cosp = Scalar{2} * (q.w() * q.x() + q.y() * q.z());
  Scalar cosr_cosp = Scalar{1} - Scalar{2} * (q.x() * q.x() + q.y() * q.y());
  angles.x() = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  Scalar sinp = sqrt(Scalar{1} + Scalar{2} * (q.w() * q.y() - q.x() * q.z()));
  Scalar cosp = sqrt(Scalar{1} - Scalar{2} * (q.w() * q.y() - q.x() * q.z()));
  angles.y() =
      Scalar{2} * atan2(sinp, cosp) - numbers::pi_v<Scalar> / Scalar{2};

  // yaw (z-axis rotation)
  Scalar siny_cosp = Scalar{2} * (q.w() * q.z() + q.x() * q.y());
  Scalar cosy_cosp = Scalar{1} - Scalar{2} * (q.y() * q.y() + q.z() * q.z());
  angles.z() = atan2(siny_cosp, cosy_cosp);

  return angles;
}

}  // namespace fsc
#endif  // FSC_AUTOPILOT_MATH_ROTATION_HPP_
