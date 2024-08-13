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

#ifndef FSC_AUTOPILOT_MATH_NUMBERS_HPP_
#define FSC_AUTOPILOT_MATH_NUMBERS_HPP_

#include <type_traits>

namespace fsc::numbers {

// NOLINTBEGIN(readability-identifier-naming)

namespace details {

template <typename Scalar>
using FltOnly = std::enable_if_t<std::is_floating_point_v<Scalar>, Scalar>;
}

template <typename Scalar>
inline constexpr Scalar pi_v =
    static_cast<details::FltOnly<Scalar>>(3.14159265358979323846L);

template <typename Scalar>
inline constexpr Scalar sqrt2_v =
    static_cast<details::FltOnly<Scalar>>(1.41421356237309504880L);

template <typename Scalar>
inline constexpr Scalar std_gravity_v =
    static_cast<details::FltOnly<Scalar>>(9.80665L);

template <typename Scalar>
inline constexpr Scalar rad_per_deg_v =
    static_cast<details::FltOnly<Scalar>>(0.017453292519943295L);

template <typename Scalar>
inline constexpr Scalar deg_per_rad_v =
    static_cast<details::FltOnly<Scalar>>(57.2957795130823229L);

inline constexpr double pi = pi_v<double>;

inline constexpr double sqrt2 = sqrt2_v<double>;

inline constexpr double std_gravity = std_gravity_v<double>;

inline constexpr double rad_per_deg = rad_per_deg_v<double>;

inline constexpr double deg_per_rad = deg_per_rad_v<double>;

// NOLINTEND(readability-identifier-naming)
}  // namespace fsc::numbers

#endif  // FSC_AUTOPILOT_MATH_NUMBERS_HPP_
