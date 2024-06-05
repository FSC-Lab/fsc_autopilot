#ifndef TRACKING_CONTROL_ROTOR_DRAG_MODEL_HPP_
#define TRACKING_CONTROL_ROTOR_DRAG_MODEL_HPP_

#include "Eigen/Dense"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"

namespace control {

template <typename Derived>
struct RotorDragModel {
  using Scalar = typename ControllerTraits<Derived>::Scalar;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  inline static const Vector3 kGravity{Vector3::UnitZ() * Scalar(9.81)};

  const Derived& derived() const { return static_cast<const Derived&>(*this); }
  const Matrix3& drag_matrix() const { return derived().drag_matrix(); }

  Vector3 computeDrag(const Vector3& velocity_ref,
                      const Vector3& acceleration_ref, Scalar yaw_ref) const {
    const auto r_ref = details::accelerationVectorToRotation(
        acceleration_ref + kGravity, yaw_ref);
    return r_ref * drag_matrix() * r_ref.transpose() * velocity_ref;
  }
};

}  // namespace control

#endif  // TRACKING_CONTROL_ROTOR_DRAG_MODEL_HPP_
