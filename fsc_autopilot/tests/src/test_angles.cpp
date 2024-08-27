// Copyright © 20.024 FSC Lab
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

#include "fsc_autopilot/math/math_extras.hpp"
#include "gtest/gtest.h"

TEST(TestAngles, testWrapTo2Pi) {
  constexpr double kEpsilon = 1e-9;
  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(0.0), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapTo2Pi(fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(2 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapTo2Pi(3 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(4 * fsc::numbers::pi), kEpsilon);

  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(-0.0), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapTo2Pi(-fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(-2 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapTo2Pi(-3 * fsc::numbers::pi),
              kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(-4 * fsc::numbers::pi), kEpsilon);

  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(-0.0), kEpsilon);
  EXPECT_NEAR(3 * fsc::numbers::pi / 2, fsc::wrapTo2Pi(-fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapTo2Pi(-fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapTo2Pi(-3 * fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(-4 * fsc::numbers::pi / 2), kEpsilon);

  EXPECT_NEAR(0.0, fsc::wrapTo2Pi(0.0), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapTo2Pi(fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapTo2Pi(5 * fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapTo2Pi(9 * fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapTo2Pi(-3 * fsc::numbers::pi / 2),
              kEpsilon);
}

TEST(TestAngles, testWrapToPi) {
  constexpr double kEpsilon = 1e-9;
  EXPECT_NEAR(0.0, fsc::wrapToPi(0.0), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapToPi(fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapToPi(2 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapToPi(3 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapToPi(4 * fsc::numbers::pi), kEpsilon);

  EXPECT_NEAR(0.0, fsc::wrapToPi(-0.0), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapToPi(-fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapToPi(-2 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapToPi(-3 * fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapToPi(-4 * fsc::numbers::pi), kEpsilon);

  EXPECT_NEAR(0.0, fsc::wrapToPi(-0.0), kEpsilon);
  EXPECT_NEAR(-fsc::numbers::pi / 2, fsc::wrapToPi(-fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi, fsc::wrapToPi(-fsc::numbers::pi), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapToPi(-3 * fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(0.0, fsc::wrapToPi(-4 * fsc::numbers::pi / 2), kEpsilon);

  EXPECT_NEAR(0.0, fsc::wrapToPi(0.0), kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapToPi(fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapToPi(5 * fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapToPi(9 * fsc::numbers::pi / 2),
              kEpsilon);
  EXPECT_NEAR(fsc::numbers::pi / 2, fsc::wrapToPi(-3 * fsc::numbers::pi / 2),
              kEpsilon);
}
