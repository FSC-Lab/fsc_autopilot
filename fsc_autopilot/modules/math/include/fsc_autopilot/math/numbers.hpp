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
inline constexpr Scalar std_gravity_v =
    static_cast<details::FltOnly<Scalar>>(9.80665L);

template <typename Scalar>
inline constexpr Scalar rad_per_deg_v =
    static_cast<details::FltOnly<Scalar>>(0.017453292519943295L);

template <typename Scalar>
inline constexpr Scalar deg_per_rad_v =
    static_cast<details::FltOnly<Scalar>>(57.2957795130823229L);

inline constexpr double pi = pi_v<double>;

inline constexpr double std_gravity = std_gravity_v<double>;

inline constexpr double rad_per_deg = rad_per_deg_v<double>;

inline constexpr double deg_per_rad = deg_per_rad_v<double>;

// NOLINTEND(readability-identifier-naming)
}  // namespace fsc::numbers

#endif  // FSC_AUTOPILOT_MATH_NUMBERS_HPP_
