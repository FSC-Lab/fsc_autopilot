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

#include <random>

#include "fsc_autopilot/math/polynomial.hpp"
#include "gtest/gtest.h"

namespace {
std::random_device dev;
std::mt19937 rng{dev()};
std::uniform_real_distribution<> dist{-1000.0, 1000.0};
}  // namespace

constexpr int kNumTrials = 100;
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
