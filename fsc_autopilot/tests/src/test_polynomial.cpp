#include <random>

#include "fsc_autopilot/math/polynomial.hpp"
#include "gtest/gtest.h"

namespace {
std::random_device dev;
std::mt19937 rng{dev()};
std::uniform_real_distribution<> dist{-1000.0, 1000.0};
}  // namespace

constexpr int kNumTrials = 1000;
constexpr int kNumDataPoints = 1000;

TEST(TestPolynomial, testZeroCoefficients) {
  math::Polynomial<double> poly;
  for (int i = 0; i < kNumTrials; ++i) {
    ASSERT_EQ(poly.vals(dist(rng)), 0.0);
  }
}

TEST(TestPolynomial, testLinear) {
  for (int i = 0; i < kNumTrials; ++i) {
    auto slope = dist(rng);
    auto intercept = dist(rng);
    math::Polynomial<double> poly{Eigen::Vector2d{slope, intercept}};

    const Eigen::ArrayXd xv =
        Eigen::ArrayXd::LinSpaced(kNumDataPoints, -1000.0, 1000.0);

    const Eigen::VectorXd yv_expected = slope * xv + intercept;
    const Eigen::VectorXd yv_result =
        xv.unaryExpr([&poly](auto&& v) { return poly.vals(v); });

    ASSERT_TRUE(yv_expected.isApprox(yv_result));
  }
}

TEST(TestPolynomial, testQuadratic) {
  for (int i = 0; i < kNumTrials; ++i) {
    auto a = dist(rng);
    auto b = dist(rng);
    auto c = dist(rng);
    math::Polynomial<double> poly{Eigen::Vector3d{a, b, c}};

    const Eigen::ArrayXd xv =
        Eigen::ArrayXd::LinSpaced(kNumDataPoints, -1000.0, 1000.0);

    const Eigen::VectorXd yv_expected = a * xv.square() + b * xv + c;
    const Eigen::VectorXd yv_result =
        xv.unaryExpr([&poly](auto&& v) { return poly.vals(v); });

    ASSERT_TRUE(yv_expected.isApprox(yv_result));
  }
}

TEST(TestPolynomial, testCubic) {
  for (int i = 0; i < kNumTrials; ++i) {
    auto a = dist(rng);
    auto b = dist(rng);
    auto c = dist(rng);
    auto d = dist(rng);
    math::Polynomial<double> poly{Eigen::Vector4d{a, b, c, d}};

    const Eigen::ArrayXd xv =
        Eigen::ArrayXd::LinSpaced(kNumDataPoints, -1000.0, 1000.0);

    const Eigen::VectorXd yv_expected =
        a * xv.cube() + b * xv.square() + c * xv + d;
    const Eigen::VectorXd yv_result =
        xv.unaryExpr([&poly](auto&& v) { return poly.vals(v); });

    ASSERT_TRUE(yv_expected.isApprox(yv_result));
  }
}
