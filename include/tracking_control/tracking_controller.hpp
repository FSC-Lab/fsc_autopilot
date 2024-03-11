#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include "Eigen/Dense"
#include "tracking_control/acceleration_setpoint_shaping.hpp"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/rotor_drag_model.hpp"

namespace control {
template <typename Scalar>
class TrackingController;

template <typename Scalar>
struct TrackingControllerState {
  Vector3<Scalar> position;
  Vector3<Scalar> velocity;
};

template <typename Scalar>
struct TrackingControllerReference {
  Vector3<Scalar> position;
  Vector3<Scalar> velocity;
  Vector3<Scalar> acceleration;
  Scalar yaw;
};

template <typename Scalar>
struct TrackingControllerError {
  Vector3<Scalar> position_error;
  Vector3<Scalar> velocity_error;
};

template <typename Scalar>
struct TrackingControllerOutput {
  Scalar thrust;
  Quaternion<Scalar> orientation;
};

template <typename Scalar>
struct TrackingControllerParameters {
  static constexpr Scalar kDefaultKpXY{1.0};
  static constexpr Scalar kDefaultKpZ{10.0};
  Vector3<Scalar> k_pos{kDefaultKpXY, kDefaultKpXY, kDefaultKpZ};
  static constexpr Scalar kDefaultKvXY{1.5};
  static constexpr Scalar kDefaultKvZ{3.3};
  Vector3<Scalar> k_vel{kDefaultKvXY, kDefaultKvXY, kDefaultKvZ};
  Matrix3<Scalar> drag_d{Matrix3<Scalar>::Zero()};
  Scalar min_z_accel{0};
  static constexpr Scalar kDefaultMaxZAccel{20};
  Scalar max_z_accel{kDefaultMaxZAccel};
  static constexpr Scalar kDefaultMaxTiltAngle{45};
  Scalar max_tilt_ratio{std::tan(details::deg2rad(kDefaultMaxTiltAngle))};
};

template <typename S>
struct ControllerTraits<TrackingController<S>> {
  using Scalar = S;
  using State = TrackingControllerState<Scalar>;
  using Reference = TrackingControllerReference<Scalar>;
  using Error = TrackingControllerError<Scalar>;
  using Output = TrackingControllerOutput<Scalar>;
  using Parameters = TrackingControllerParameters<Scalar>;
};

template <typename T>
class TrackingController
    : public ControllerBase<TrackingController<T>>,
      public RotorDragModel<TrackingController<T>>,
      public AccelerationSetpointShaping<TrackingController<T>> {
 public:
  using Scalar = T;
  using Base = ControllerBase<TrackingController<Scalar>>;

  using Vector3Type = Vector3<Scalar>;
  using QuaternionType = Quaternion<Scalar>;

  using State = typename Base::State;
  using Reference = typename Base::Reference;
  using Result = typename Base::Result;
  using Parameters = typename Base::Parameters;
  inline static const Vector3Type kGravity{Vector3Type::UnitZ() * Scalar(9.81)};

  using RotorDragModel<TrackingController>::computeDrag;
  using AccelerationSetpointShaping<
      TrackingController>::reshapeAccelerationSetpoint;

  Result runImpl(const State& state, const Reference& refs) const {
    using std::atan2;

    auto position_error = state.position - refs.position;
    auto velocity_error = state.velocity - refs.velocity;

    const auto feedback = params_.k_pos.cwiseProduct(position_error) +
                          params_.k_vel.cwiseProduct(velocity_error);

    const auto rotor_drag =
        computeDrag(refs.velocity, refs.acceleration, refs.yaw);
    const auto accel_sp = reshapeAccelerationSetpoint(
        -feedback - rotor_drag + kGravity + refs.acceleration);
    auto attitude_sp =
        details::accelerationVectorToRotation(accel_sp, refs.yaw);

    auto thrust_sp = accel_sp.dot(attitude_sp * Vector3Type::UnitZ());

    return {{thrust_sp, Quaternion<Scalar>(attitude_sp)},
            {position_error, velocity_error}};
  }

  const Parameters& params() const { return params_; }
  Parameters& params() { return params_; }

  // Expose this member for RotorDragModel mixin
  const Matrix3<Scalar>& drag_matrix() const { return params_.drag_d; }

  // Expose these parameters for AccelerationSetpointShaping mixin
  Scalar min_z_accel() const { return params_.min_z_accel; }
  Scalar max_z_accel() const { return params_.max_z_accel; }
  Scalar max_tilt_ratio() const { return params_.max_tilt_ratio; }

 private:
  Parameters params_;
};
}  // namespace control

#endif  // TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
