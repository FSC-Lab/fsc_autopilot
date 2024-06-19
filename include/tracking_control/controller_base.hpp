#ifndef TRACKING_CONTROL_CONTROLLER_BASE_HPP_
#define TRACKING_CONTROL_CONTROLLER_BASE_HPP_

#include "Eigen/Dense"

namespace control {

template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
template <typename Scalar>
using Quaternion = Eigen::Quaternion<Scalar>;

template <typename Derived>
struct ControllerTraits {};

template <typename Derived>
class ControllerBase {
 public:
  using Scalar = typename ControllerTraits<Derived>::Scalar;
  using State = typename ControllerTraits<Derived>::State;
  using Reference = typename ControllerTraits<Derived>::Reference;
  using Output = typename ControllerTraits<Derived>::Output;
  using Error = typename ControllerTraits<Derived>::Error;
  using Parameters = typename ControllerTraits<Derived>::Parameters;

  struct Result {
    bool success{false};
    Output output;
    Error error;
  };

  const Derived& derived() const { return static_cast<const Derived&>(*this); }
  Derived& derived() { return static_cast<Derived&>(*this); }

  const Parameters& params() const { return derived().params(); }
  Parameters& params() { return derived().params(); }

  Result run(const State& state, const Reference& refs, double dt,
             bool intFlag) {
    return derived().runImpl(state, refs, dt, intFlag);
  }
};
}  // namespace control

#endif  // TRACKING_CONTROL_CONTROLLER_BASE_HPP_
