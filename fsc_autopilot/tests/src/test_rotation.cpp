// Copyright © 2023 FSC
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

#include <limits>
#include <random>

#include "Eigen/Dense"
#include "fsc_autopilot/math/numbers.hpp"
#include "fsc_autopilot/math/rotation.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

static const Eigen::IOFormat kFmt(Eigen::StreamPrecision, 0, ",", ";\n", "", "",
                                  "[", "]");
static constexpr int kNumTrials = 10000;
static constexpr double kSmallAngle = 1e-2;
static inline const double kTinyAngle =
    std::pow(std::numeric_limits<double>::min(), 0.75);
static constexpr double kHalfSqrt2 = fsc::numbers::sqrt2 / 2.0;
static constexpr double kPi = fsc::numbers::pi;
static constexpr double kHalfPi = kPi / 2.0;

template <typename Derived1, typename Derived2>
bool AllClose(const Eigen::MatrixBase<Derived1>& lhs,
              const Eigen::MatrixBase<Derived2>& rhs) {
  return lhs.binaryExpr(rhs, [](auto a, auto b) { return fsc::IsClose(a, b); })
      .array()
      .all();
}

MATCHER_P(QuaternionIsClose, expected, ::testing::PrintToString(expected)) {
  return fsc::IsClose(arg.angularDistance(expected), 0.0);
}

MATCHER_P(AllClose, expected,
          testing::PrintToString(expected.transpose().format(kFmt))) {
  return AllClose(arg, expected);
}

MATCHER_P(AngleAxisIsClose, expected,
          ::testing::PrintToString(expected.transpose().format(kFmt))) {
  if (fsc::IsClose(expected.norm(), fsc::numbers::pi)) {
    return AllClose(arg, expected) || AllClose(arg, -expected);
  }
  return AllClose(arg, expected);
}

MATCHER(IsOrthogonal,
        "Matrix must be orthogonal, i.e. itself multiplied by its transpose is "
        "equal to the identity matrix") {
  constexpr auto kIsOrthogonalTol = 1e-8;
  return ((arg * arg.transpose()).isIdentity(kIsOrthogonalTol) &&
          (arg.transpose() * arg).isIdentity(kIsOrthogonalTol));
}

MATCHER(IsNormalized, "Expected unit norm") {
  return fsc::IsClose(arg.norm(), 1.0);
}

class TestRotation : public ::testing::Test {
 public:
  using ::testing::Test::Test;

  std::mt19937& rng() { return rng_; }

  double random() { return dist_(rng_); }

  double random(double low, double high) {
    using ParamType = std::uniform_real_distribution<double>::param_type;
    return dist_(rng_, ParamType{low, high});
  }

  Eigen::Vector3d generateRandomVector() {
    return Eigen::Vector3d::NullaryExpr([this] { return random(); });
  }

  Eigen::Vector3d generateRandomUnitVector() {
    return generateRandomVector().normalized();
  }

  Eigen::Quaterniond generateRandomQuaternion() {
    const double u1 = random(0.0, 1.0);
    const double u2 = random(0.0, 2.0 * kPi);
    const double u3 = random(0.0, 2.0 * kPi);
    const double a = sqrt(1 - u1);
    const double b = sqrt(u1);
    return {a * std::sin(u2),  //
            a * std::cos(u2),  //
            b * std::sin(u3),  //
            b * std::cos(u3)};
  }

 private:
  std::random_device dev_;
  std::mt19937 rng_{dev_()};
  std::uniform_real_distribution<> dist_{-1.0, 1.0};
};

TEST_F(TestRotation, ZeroAngleAxisToQuaternion) {
  const Eigen::Vector3d axis_angle = Eigen::Vector3d::Zero();

  const Eigen::Quaterniond result = fsc::AngleAxisToQuaternion(axis_angle);
  const Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expected));
}

TEST_F(TestRotation, TinyAngleAxisToQuaternion) {
  // Very small value that could potentially cause underflow.
  const Eigen::Vector3d axis = generateRandomUnitVector();
  const Eigen::Vector3d axis_angle = kTinyAngle * axis;

  const Eigen::Quaterniond expected(Eigen::AngleAxisd(kTinyAngle, axis));
  const Eigen::Quaterniond result = fsc::AngleAxisToQuaternion(axis_angle);
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expected));
}

TEST_F(TestRotation, SmallAngleAxisToQuaternion) {
  // Small, finite value to test.
  const Eigen::Vector3d axis = generateRandomUnitVector();
  const Eigen::Vector3d axis_angle = kSmallAngle * axis;

  const Eigen::Quaterniond expected(Eigen::AngleAxisd(kSmallAngle, axis));
  const Eigen::Quaterniond result = fsc::AngleAxisToQuaternion(axis_angle);
  ASSERT_THAT(result, IsNormalized());
  ASSERT_THAT(result, QuaternionIsClose(expected));
}

TEST_F(TestRotation, AngleAxisToQuaternion) {
  for (int i = 0; i < 3; ++i) {
    auto axis = Eigen::Vector3d::Unit(i);
    auto axis_angle = kHalfPi * axis;

    Eigen::Quaterniond expected;
    expected.w() = kHalfSqrt2;
    expected.vec() = kHalfSqrt2 * axis;
    const Eigen::Quaterniond result = fsc::AngleAxisToQuaternion(axis_angle);
    ASSERT_THAT(result, IsNormalized());
    ASSERT_THAT(result, QuaternionIsClose(expected));
  }
}

// Transforms a unit quaternion to an axis angle.
TEST_F(TestRotation, UnitQuaternionToAngleAxis) {
  const Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();

  const Eigen::Vector3d expected = Eigen::Vector3d::Zero();
  const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AllClose(expected));
}

TEST_F(TestRotation, TinyQuaternionToAngleAxis) {
  // Very small value that could potentially cause underflow.
  const Eigen::Vector3d axis = generateRandomUnitVector();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(kTinyAngle, axis));

  const Eigen::Vector3d expected = kTinyAngle * axis;
  const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AngleAxisIsClose(expected));
}

TEST_F(TestRotation, SmallQuaternionToAngleAxis) {
  // Small, finite value to test.
  const Eigen::Vector3d axis = generateRandomUnitVector();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(kSmallAngle, axis));

  const Eigen::Vector3d expected = kSmallAngle * axis;
  const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(quaternion);
  ASSERT_THAT(result, AngleAxisIsClose(expected));
}

TEST_F(TestRotation, QuaternionToAngleAxis) {
  Eigen::Quaterniond quaternion;

  for (int i = 0; i < 3; ++i) {
    quaternion.w() = std::sqrt(3) / 2;
    quaternion.vec() = Eigen::Vector3d::Unit(i) * 0.5;

    const Eigen::Vector3d expected = Eigen::Vector3d::Unit(i) * kPi / 3;
    const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(quaternion);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }

  for (int i = 0; i < 3; ++i) {
    quaternion.w() = 0.0;
    quaternion.vec() = Eigen::Vector3d::Unit(i);

    const Eigen::Vector3d expected = Eigen::Vector3d::Unit(i) * kPi;
    const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(quaternion);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, ZeroAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis_angle = Eigen::Vector3d::Zero();

  const Eigen::Matrix3d result = fsc::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expected));
}

TEST_F(TestRotation, TinyAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis_angle(1e-24, 2e-24, 3e-24);

  const Eigen::Matrix3d result = fsc::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expected));
}

TEST_F(TestRotation, SmallAngleAxisToRotationMatrix) {
  const Eigen::Vector3d axis = generateRandomUnitVector();
  const Eigen::Vector3d axis_angle = kTinyAngle * axis;

  const Eigen::Matrix3d result = fsc::AngleAxisToRotationMatrix(axis_angle);
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(kTinyAngle, axis).toRotationMatrix();
  ASSERT_THAT(result, IsOrthogonal());
  ASSERT_THAT(result, AllClose(expected));
}

// Transforms a rotation by pi/2 around X to a rotation matrix and back.
TEST_F(TestRotation, AngleAxisToRotationMatrix) {
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angle_axis = kPi / 2 * Eigen::Vector3d::Unit(i);
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    Eigen::Matrix3d expected;
    expected(i, i) = 1.0;
    expected(i, j) = expected(i, k) = expected(j, i) = expected(k, i) = 0.0;
    expected(j, j) = expected(k, k) = 0.0;
    expected(j, k) = -1.0;
    expected(k, j) = 1.0;
    const Eigen::Matrix3d result = fsc::AngleAxisToRotationMatrix(angle_axis);
    ASSERT_THAT(result, IsOrthogonal());
    ASSERT_THAT(result, AllClose(expected));
  }

  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d angle_axis = kPi * Eigen::Vector3d::Unit(i);
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    Eigen::Matrix3d expected;
    expected(i, i) = 1.0;
    expected(i, j) = expected(i, k) = expected(j, i) = expected(k, i) = 0.0;
    expected(j, k) = expected(k, j) = 0.0;
    expected(j, j) = -1.0;
    expected(k, k) = -1.0;
    const Eigen::Matrix3d result = fsc::AngleAxisToRotationMatrix(angle_axis);
    ASSERT_THAT(result, IsOrthogonal());
    ASSERT_THAT(result, AllClose(expected));
  }
}

TEST_F(TestRotation, QuaternionToAngleAxisAngleIsLessThanPi) {
  const double half_theta = 0.75 * kPi;
  Eigen::Quaterniond quaternion;
  quaternion.x() = cos(half_theta);
  quaternion.y() = sin(half_theta);
  quaternion.z() = 0.0;
  quaternion.w() = 0.0;
  auto angle_axis = fsc::QuaternionToAngleAxis(quaternion);
  const double angle = angle_axis.norm();
  ASSERT_LE(angle, kPi);
}

TEST_F(TestRotation, AngleAxisToQuaternionAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Vector3d expected =
        kPi * random() * generateRandomUnitVector();
    const Eigen::Quaterniond intermediate =
        fsc::AngleAxisToQuaternion(expected);
    ASSERT_THAT(intermediate, IsNormalized());
    const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, QuaternionToAngleAxisAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    Eigen::Quaterniond expected;

    const Eigen::Vector3d intermediate = fsc::QuaternionToAngleAxis(expected);
    const Eigen::Quaterniond result = fsc::AngleAxisToQuaternion(intermediate);
    ASSERT_THAT(result, IsNormalized());
    ASSERT_THAT(result, QuaternionIsClose(expected));
  }
}

TEST_F(TestRotation, AngleAxisToRotationMatrixAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    auto angle = kPi * random();
    const Eigen::Vector3d expected = angle * generateRandomUnitVector();
    const Eigen::Matrix3d intermediate =
        fsc::AngleAxisToRotationMatrix(expected);
    ASSERT_THAT(intermediate, IsOrthogonal());
    const Eigen::Vector3d result = fsc::RotationMatrixToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, RotationMatrixToAngleAxisAndBack) {
  for (int i = 0; i < kNumTrials; i++) {
    const Eigen::Quaterniond expected_quaternion = generateRandomQuaternion();
    const Eigen::Matrix3d expected = expected_quaternion.toRotationMatrix();
    const Eigen::Vector3d intermediate =
        fsc::RotationMatrixToAngleAxis(expected);
    const Eigen::Matrix3d result = fsc::AngleAxisToRotationMatrix(intermediate);
    ASSERT_THAT(result, AllClose(expected));
  }
}

// Takes a bunch of random axis/angle values, converts them to quaternions,
// and back again.
TEST_F(TestRotation, NearPiAngleAxisToQuaternionAndBack) {
  Eigen::Quaterniond intermediate;
  constexpr double kMaxSmallAngle = 1e-8;
  for (int i = 0; i < kNumTrials; i++) {
    const double theta = kPi - kMaxSmallAngle * random();
    const Eigen::Vector3d expected = theta * generateRandomUnitVector();
    intermediate = fsc::AngleAxisToQuaternion(expected);
    ASSERT_THAT(intermediate, IsNormalized());
    const Eigen::Vector3d result = fsc::QuaternionToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, NearPiAngleAxisToRotationMatrixAndBack) {
  constexpr double kMaxSmallAngle = 1e-8;
  for (int i = 0; i < kNumTrials; i++) {
    const double theta = kPi - kMaxSmallAngle * random();
    const Eigen::Vector3d expected = theta * generateRandomUnitVector();

    const Eigen::Matrix3d intermediate =
        fsc::AngleAxisToRotationMatrix(expected);
    const Eigen::Vector3d result = fsc::RotationMatrixToAngleAxis(intermediate);
    ASSERT_THAT(result, AngleAxisIsClose(expected));
  }
}

TEST_F(TestRotation, NearPiRotationMatrixToAngleAxisAndBack) {
  constexpr double kMaxSmallAngle = 1e-8;
  for (int a = 0; a < (kNumTrials / 3); ++a) {
    for (int i = 0; i < 3; ++i) {
      const double theta = kPi - kMaxSmallAngle * random();
      const double st = sin(theta);
      const double ct = cos(theta);
      const int j = (i + 1) % 3;
      const int k = (j + 1) % 3;
      Eigen::Matrix3d expected;
      expected(i, i) = 1.0;
      expected(i, j) = expected(i, k) = expected(j, i) = expected(k, i) = 0.0;
      expected(j, k) = st;
      expected(k, j) = -st;
      expected(j, j) = expected(k, k) = -ct;
      const Eigen::Vector3d intermediate =
          fsc::RotationMatrixToAngleAxis(expected);
      const Eigen::Matrix3d result =
          fsc::AngleAxisToRotationMatrix(intermediate);
      ASSERT_THAT(result, AllClose(expected));
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
