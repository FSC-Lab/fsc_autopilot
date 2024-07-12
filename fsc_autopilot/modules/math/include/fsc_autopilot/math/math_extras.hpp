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

#ifndef FSC_AUTOPILOT_MATH_MATH_EXTRAS_HPP_
#define FSC_AUTOPILOT_MATH_MATH_EXTRAS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "fsc_autopilot/math/numbers.hpp"

namespace fsc {

template <typename T>
struct Tolerances {
  // NOLINTBEGIN(readability-magic-numbers)
  T relative{T(1e-5)};
  T absolute{T(1e-8)};
  // NOLINTEND(readability-magic-numbers)
};

/**
 * @brief Robust floating value comparison function
 *
 * @details This implementation is lifted from
 *
 * https://stackoverflow.com/questions/4915462/how-should-i-do-floating-point-comparison
 *
 * Note that calling IsClose(value, 0) is NOT substantially slower than a
 * hypothetical IsZero() function implementing value < absolute_tolerance
 *
 * @param a Floating point value
 * @param b Floating point value
 * @param tols The tolerances struct containg absolute and relative tolerance
 * values
 */
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

/**
 * @brief Optimized function to check if two values are the same sign
 *
 * @param x Left operand
 * @param y Right operand
 * @return true Two values have the same sign
 * @return false Two values have different signs
 */
template <typename T>
bool IsSameSign(const T& x, const T& y) {
  return (x >= 0) ^ (y < 0);
}

/**
 * @brief Floating-point modulo modelled after MATLAB's `mod()`, but not
 * `fmod()`. The result (the remainder) has same sign as the divisor. Note that
 * `mod(-3,4) = 1` but `fmod(-3,4) = -3`
 *
 * @param x Left operand
 * @param y Right operand
 * @param tols The tolerances struct containg absolute and relative tolerances
 * used to handle boundary cases
 * @return T modulus after division
 */
template <typename T>
constexpr T Mod(const T& x, const T& y,
                const Tolerances<T>& tols = Tolerances<T>()) {
  static_assert(std::is_floating_point_v<T>, "EXPECTED_FLOATING_POINT_NUMBERS");
  using std::floor;
  if (IsClose(y, T(0), tols)) {
    return x;
  }
  const T m = x - y * floor(x / y);

  // handle boundary cases resulted from floating-point cut off:
  if (y > T(0)) {
    if (m >= y) {
      return 0;
    }

    if (m < T(0)) {
      return IsClose(y + m, y, tols) ? T(0) : y + m;
    }
  } else {
    if (m <= y) {
      return 0;
    }

    if (m > T(0)) {
      return IsClose(y + m, y, tols) ? T(0) : y + m;
    }
  }

  return m;
}

/**
 * @brief Wrap angle in radians to [-pi, pi]
 *
 * @param angle T Angle in radians
 * @param tols The tolerances struct containg absolute and relative tolerances
 * used to handle edge values
 * @return constexpr T Angle in [-pi, pi]
 */
template <typename T>
constexpr T wrapToPi(T angle) {
  using std::fmod;
  const T result = fmod(angle + numbers::pi_v<T>, 2.0 * numbers::pi_v<T>);
  return result <= 0.0 ? result + numbers::pi_v<T> : result - numbers::pi_v<T>;
}
/**
 * @brief Wrap angle in radians to [0, 2 pi]
 *
 * @param angle Angle in radians
 * @param tols The tolerances struct containg absolute and relative tolerances
 * used to handle edge values
 * @return constexpr T Angle in [0, 2 pi]
 */
template <typename T>
constexpr T wrapTo2Pi(T angle) {
  using std::fmod;
  T res = fmod(angle, T{2} * numbers::pi_v<T>);
  return res < T{0} ? res + T{2} * numbers::pi_v<T> : res;
}

/**
 * @brief Wrap angle in degrees to [-180, 180]
 *
 * @param angle Angle in degrees
 * @param tols The tolerances struct containg absolute and relative tolerances
 * used to handle edge values
 * @return constexpr T Angle in [-180, 180]
 */
template <typename T>
constexpr T WrapTo180(T angle, const Tolerances<T>& tols = Tolerances<T>()) {
  using std::fmod;
  const T res = fmod(angle + T(180), T(360), tols);
  return res <= T(0) ? res + T(180) : res - T(180);
}

/**
 * @brief Wrap angle in degrees to [0, 360]
 *
 * @param angle Angle in degrees
 * @param tols The tolerances struct containg absolute and relative tolerances
 * used to handle edge values
 * @return constexpr T Angle in [0, 360]
 */
template <typename T>
constexpr T WrapTo360(T angle, const Tolerances<T>& tols = Tolerances<T>()) {
  using std::fmod;
  T res = fmod(angle, T(360), tols);
  return res < T{0} ? res + T{360} : res;
}

/**
 * @brief Convert angle in degrees to angle in radians
 *
 * @param deg Angle in degrees
 * @return constexpr T Angle in radians
 */
template <typename T>
constexpr T deg2rad(T deg) noexcept {
  return numbers::pi_v<T> * deg / T(180);
}

/**
 * @brief Convert angle in radians to angle in degrees
 *
 * @param rad Angle in radians
 * @return constexpr T Angle in degrees
 */
template <typename T>
constexpr T rad2deg(T rad) noexcept {
  return T(180) / numbers::pi_v<T> * rad;
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

}  // namespace fsc

#endif  // FSC_AUTOPILOT_MATH_MATH_EXTRAS_HPP_
