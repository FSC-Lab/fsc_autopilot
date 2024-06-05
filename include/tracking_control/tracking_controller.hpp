#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include "Eigen/Dense"
#include "fmt/format.h"
#include "fmt/ostream.h"
#include "tracking_control/acceleration_setpoint_shaping.hpp"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/rotor_drag_model.hpp"

namespace control {
template <typename Scalar>
class TrackingController;

template <typename Scalar>
struct TrackingControllerState {
  Vector3<Scalar> position{Vector3<Scalar>::Zero()};
  Quaternion<Scalar> orientation{Quaternion<Scalar>::Identity()};
  Vector3<Scalar> velocity{Vector3<Scalar>::Zero()};
  Vector3<Scalar> acceleration{Vector3<Scalar>::Zero()};
};

template <typename Scalar>
struct TrackingControllerReference {
  Vector3<Scalar> position{Vector3<Scalar>::Zero()};
  Vector3<Scalar> velocity{Vector3<Scalar>::Zero()};
  Vector3<Scalar> acceleration{Vector3<Scalar>::Zero()};
  Scalar yaw{0};
};

template <typename Scalar>
struct TrackingControllerError {
  Vector3<Scalar> position_error{Vector3<Scalar>::Zero()};
  Vector3<Scalar> velocity_error{Vector3<Scalar>::Zero()};
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

  static constexpr Scalar kDefaultDEGain{5.0};
  Scalar de_gain{kDefaultDEGain};

  static constexpr Scalar kDefaultDEHeightThreshold{0.1};
  Scalar de_height_threshold{kDefaultDEHeightThreshold};

  static constexpr Scalar kVehicleMassSentinel{-1.0};
  Scalar vehicle_mass{kVehicleMassSentinel};

  static constexpr Scalar kDefaultDEBounds{5};
  Vector3<Scalar> de_lb{Vector3<Scalar>::Constant(-kDefaultDEBounds)};
  Vector3<Scalar> de_ub{Vector3<Scalar>::Constant(kDefaultDEBounds)};
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

  Result runImpl(const State& state, const Reference& refs) {
    using std::atan2;
    auto position_error = state.position - refs.position;
    auto velocity_error = state.velocity - refs.velocity;

    if (params_.vehicle_mass < Scalar(0)) {
      return {false,
              {Scalar(0), Quaternion<Scalar>::Identity()},
              {position_error, velocity_error}};
    }

    const auto feedback = params_.k_pos.cwiseProduct(position_error) +
                          params_.k_vel.cwiseProduct(velocity_error);

    const auto rotor_drag =
        computeDrag(refs.velocity, refs.acceleration, refs.yaw);

    if (refs.position.z() > params_.de_height_threshold) {
      // R_ib * [0,0,1] * f_b
      const auto expected_thrust =
          state.orientation * Vector3<Scalar>::UnitZ() * thrust_sp_;

      constexpr Scalar kStandardGravity{-9.81};
      // m * g * [0,0,1]
      const auto vehicle_weight =
          params_.vehicle_mass * kStandardGravity * Vector3<Scalar>::UnitZ();

      // R_ib * a_b * m
      const auto inertial_force =
          state.orientation * refs.acceleration * params_.vehicle_mass;
      disturbance_estimate_ =
          -params_.de_gain * (disturbance_estimate_ + expected_thrust +
                              vehicle_weight - inertial_force);

      // Bail on insane bounds
      if ((params_.de_lb.array() > params_.de_ub.array()).any()) {
        return {false,
                {Scalar(0), Quaternion<Scalar>::Identity()},
                {position_error, velocity_error}};
      }
      // Clamp disturbance estimate
      disturbance_estimate_ =
          disturbance_estimate_.cwiseMax(params_.de_lb).cwiseMin(params_.de_ub);
    } else {
      disturbance_estimate_.setZero();
    }

    const auto accel_sp =
        reshapeAccelerationSetpoint(-feedback - rotor_drag + kGravity +
                                    refs.acceleration + disturbance_estimate_);
    auto attitude_sp =
        details::accelerationVectorToRotation(accel_sp, refs.yaw);

    thrust_sp_ = accel_sp.dot(attitude_sp * Vector3Type::UnitZ());

    return {true,
            {thrust_sp_, Quaternion<Scalar>(attitude_sp)},
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
  Vector3<Scalar> disturbance_estimate_{Vector3<Scalar>::Zero()};

  Scalar thrust_sp_{0.0};
  Parameters params_;
};
}  // namespace control

namespace fmt {
template <typename Scalar>
struct formatter<control::TrackingControllerParameters<Scalar>> {
  constexpr auto parse(format_parse_context& ctx)
      -> format_parse_context::iterator {
    return ctx.begin();
  }

  auto format(const control::TrackingControllerParameters<Scalar>& v,
              format_context& ctx) const -> format_context::iterator {
    Eigen::IOFormat ei_fmt(Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[",
                           "]");
    return format_to(
        ctx.out(),
        "k_pos: {}\nk_vel: {}\naccel bounds: [{}, {}]\nde gain: {}\nde bounds: "
        "[{}, {}]\nde height threshold: {}",
        v.k_pos.transpose().format(ei_fmt), v.k_vel.transpose().format(ei_fmt),
        v.min_z_accel, v.max_z_accel, v.de_gain,
        v.de_lb.transpose().format(ei_fmt), v.de_ub.transpose().format(ei_fmt),
        v.de_height_threshold);
  }
};
}  // namespace fmt

#endif  // TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
