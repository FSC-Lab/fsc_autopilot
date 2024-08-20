
/**
 * Test suite to demonstrate a good method for comparing floating-point values
 * using an epsilon. Run via JUnit 4.
 *
 * Note: this function attempts a "one size fits all" solution. There may be
 * some edge cases for which it still produces unexpected results, and some of
 * the tests it was developed to pass probably specify behaviour that is not
 * appropriate for some applications, especially concerning very small values
 * with differing signs.
 *
 * Before using it, make sure it's appropriate for your application!
 *
 * From http://floating-point-gui.de
 *
 * @author Michael Borgwardt
 */

#include <limits>

#include "fsc_autopilot/math/math_extras.hpp"
#include "gtest/gtest.h"

using Limits = std::numeric_limits<float>;
auto IsClose(float a, float b, float epsilon = 1e-5F) {
  return fsc::IsClose(a, b, {epsilon, epsilon * Limits::min()});
}

/** Regular large numbers - generally not problematic */
TEST(TestIsClose, big) {
  ASSERT_TRUE(IsClose(1000000.0F, 1000001.0F));
  ASSERT_TRUE(IsClose(1000001.0F, 1000000.0F));
  ASSERT_FALSE(IsClose(10000.0F, 10001.0F));
  ASSERT_FALSE(IsClose(10001.0F, 10000.0F));
}

/** Negative large numbers */

TEST(TestIsClose, bigNeg) {
  ASSERT_TRUE(IsClose(-1000000.0F, -1000001.0F));
  ASSERT_TRUE(IsClose(-1000001.0F, -1000000.0F));
  ASSERT_FALSE(IsClose(-10000.0F, -10001.0F));
  ASSERT_FALSE(IsClose(-10001.0F, -10000.0F));
}

/** Numbers around 1 */

TEST(TestIsClose, mid) {
  ASSERT_TRUE(IsClose(1.0000001F, 1.0000002F));
  ASSERT_TRUE(IsClose(1.0000002F, 1.0000001F));
  ASSERT_FALSE(IsClose(1.0002F, 1.0001F));
  ASSERT_FALSE(IsClose(1.0001F, 1.0002F));
}

/** Numbers around -1 */

TEST(TestIsClose, midNeg) {
  ASSERT_TRUE(IsClose(-1.000001F, -1.000002F));
  ASSERT_TRUE(IsClose(-1.000002F, -1.000001F));
  ASSERT_FALSE(IsClose(-1.0001F, -1.0002F));
  ASSERT_FALSE(IsClose(-1.0002F, -1.0001F));
}

/** Numbers between 1 and 0 */

TEST(TestIsClose, small) {
  ASSERT_TRUE(IsClose(0.000000001000001F, 0.000000001000002F));
  ASSERT_TRUE(IsClose(0.000000001000002F, 0.000000001000001F));
  ASSERT_FALSE(IsClose(0.000000000001002F, 0.000000000001001F));
  ASSERT_FALSE(IsClose(0.000000000001001F, 0.000000000001002F));
}

/** Numbers between -1 and 0 */

TEST(TestIsClose, smallNeg) {
  ASSERT_TRUE(IsClose(-0.000000001000001F, -0.000000001000002F));
  ASSERT_TRUE(IsClose(-0.000000001000002F, -0.000000001000001F));
  ASSERT_FALSE(IsClose(-0.000000000001002F, -0.000000000001001F));
  ASSERT_FALSE(IsClose(-0.000000000001001F, -0.000000000001002F));
}

/** Small differences away from zero */

TEST(TestIsClose, smallDiffs) {
  ASSERT_TRUE(IsClose(0.3F, 0.30000003F));
  ASSERT_TRUE(IsClose(-0.3F, -0.30000003F));
}

/** Comparisons involving zero */

TEST(TestIsClose, zero) {
  ASSERT_TRUE(IsClose(0.0F, 0.0F));
  ASSERT_TRUE(IsClose(0.0F, -0.0F));
  ASSERT_TRUE(IsClose(-0.0F, -0.0F));
  ASSERT_FALSE(IsClose(0.00000001F, 0.0F));
  ASSERT_FALSE(IsClose(0.0F, 0.00000001F));
  ASSERT_FALSE(IsClose(-0.00000001F, 0.0F));
  ASSERT_FALSE(IsClose(0.0F, -0.00000001F));

  ASSERT_TRUE(IsClose(0.0F, 1e-40F, 0.01F));
  ASSERT_TRUE(IsClose(1e-40F, 0.0F, 0.01F));
  ASSERT_FALSE(IsClose(1e-40F, 0.0F, 0.000001F));
  ASSERT_FALSE(IsClose(0.0F, 1e-40F, 0.000001F));

  ASSERT_TRUE(IsClose(0.0F, -1e-40F, 0.1F));
  ASSERT_TRUE(IsClose(-1e-40F, 0.0F, 0.1F));
  ASSERT_FALSE(IsClose(-1e-40F, 0.0F, 0.00000001F));
  ASSERT_FALSE(IsClose(0.0F, -1e-40F, 0.00000001F));
}

/**
 * Comparisons involving extreme values (overflow potential)
 */

TEST(TestIsClose, extremeMax) {
  ASSERT_TRUE(IsClose(Limits::max(), Limits::max()));
  ASSERT_FALSE(IsClose(Limits::max(), -Limits::max()));
  ASSERT_FALSE(IsClose(-Limits::max(), Limits::max()));
  ASSERT_FALSE(IsClose(Limits::max(), Limits::max() / 2));
  ASSERT_FALSE(IsClose(Limits::max(), -Limits::max() / 2));
  ASSERT_FALSE(IsClose(-Limits::max(), Limits::max() / 2));
}

/**
 * Comparisons involving infinities
 */

TEST(TestIsClose, infinities) {
  ASSERT_TRUE(IsClose(Limits::infinity(), Limits::infinity()));
  ASSERT_TRUE(IsClose(-Limits::infinity(), -Limits::infinity()));
  ASSERT_FALSE(IsClose(-Limits::infinity(), Limits::infinity()));
  ASSERT_FALSE(IsClose(Limits::infinity(), Limits::max()));
  ASSERT_FALSE(IsClose(-Limits::infinity(), -Limits::max()));
}

/**
 * Comparisons involving quiet_NaN() values
 */

TEST(TestIsClose, nan) {
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), 0.0F));
  ASSERT_FALSE(IsClose(-0.0F, Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), -0.0F));
  ASSERT_FALSE(IsClose(0.0F, Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), Limits::infinity()));
  ASSERT_FALSE(IsClose(Limits::infinity(), Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), -Limits::infinity()));
  ASSERT_FALSE(IsClose(-Limits::infinity(), Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), Limits::max()));
  ASSERT_FALSE(IsClose(Limits::max(), Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), -Limits::max()));
  ASSERT_FALSE(IsClose(-Limits::max(), Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), Limits::denorm_min()));
  ASSERT_FALSE(IsClose(Limits::denorm_min(), Limits::quiet_NaN()));
  ASSERT_FALSE(IsClose(Limits::quiet_NaN(), -Limits::denorm_min()));
  ASSERT_FALSE(IsClose(-Limits::denorm_min(), Limits::quiet_NaN()));
}

/** Comparisons of numbers on opposite sides of 0 */

TEST(TestIsClose, opposite) {
  ASSERT_FALSE(IsClose(1.000000001F, -1.0F));
  ASSERT_FALSE(IsClose(-1.0F, 1.000000001F));
  ASSERT_FALSE(IsClose(-1.000000001F, 1.0F));
  ASSERT_FALSE(IsClose(1.0F, -1.000000001F));
  ASSERT_TRUE(IsClose(10 * Limits::denorm_min(), 10 * -Limits::denorm_min()));
  ASSERT_FALSE(
      IsClose(10000 * Limits::denorm_min(), 10000 * -Limits::denorm_min()));
}

/**
 * The really tricky part - comparisons of numbers very close to zero.
 */

TEST(TestIsClose, ulp) {
  ASSERT_TRUE(IsClose(Limits::denorm_min(), Limits::denorm_min()));
  ASSERT_TRUE(IsClose(Limits::denorm_min(), -Limits::denorm_min()));
  ASSERT_TRUE(IsClose(-Limits::denorm_min(), Limits::denorm_min()));
  ASSERT_TRUE(IsClose(Limits::denorm_min(), 0.0F));
  ASSERT_TRUE(IsClose(0.0F, Limits::denorm_min()));
  ASSERT_TRUE(IsClose(-Limits::denorm_min(), 0.0F));
  ASSERT_TRUE(IsClose(0.0F, -Limits::denorm_min()));

  ASSERT_FALSE(IsClose(0.000000001F, -Limits::denorm_min()));
  ASSERT_FALSE(IsClose(0.000000001F, Limits::denorm_min()));
  ASSERT_FALSE(IsClose(Limits::denorm_min(), 0.000000001F));
  ASSERT_FALSE(IsClose(-Limits::denorm_min(), 0.000000001F));
}
