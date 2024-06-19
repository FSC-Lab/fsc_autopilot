#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include <iostream>

#include "Eigen/Dense"
#include "fmt/format.h"
#include "fmt/ostream.h"
#include "tracking_control/acceleration_setpoint_shaping.hpp"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/rotor_drag_model.hpp"
#include "utils/utils.hpp"

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
  Vector3<Scalar> ude_output{Vector3<Scalar>::Zero()};
  Vector3<Scalar> accel_sp{Vector3<Scalar>::Zero()};
  bool ude_effective{false};
};

template <typename Scalar>
struct TrackingControllerOutput {
  Scalar thrustPerRotor;  // thrust per rotor
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

  static constexpr Scalar kDefaultDEGain{1.0};
  Scalar de_gain{kDefaultDEGain};

  static constexpr Scalar kDefaultDEHeightThreshold{0.1};
  Scalar de_height_threshold{kDefaultDEHeightThreshold};

  static constexpr Scalar kVehicleMassSentinel{-1.0};
  Scalar vehicle_mass{kVehicleMassSentinel};
  Vector3<Scalar> vehicle_weight;

  uint32_t num_of_rotors{4};

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

  Result runImpl(const State& state, const Reference& refs, double dt,
                 bool intFlag) {
    using std::atan2;
    TrackingControllerError<double> res;
    Vector3<Scalar> position_error = state.position - refs.position;
    Vector3<Scalar> velocity_error = state.velocity - refs.velocity;

    if (params_.vehicle_mass < Scalar(0)) {
      return {false,
              {Scalar(0), Quaternion<Scalar>::Identity()},
              {position_error, velocity_error}};
    }

    // bound the velocity and position error
    // kv * (ev + kp * sat(ep))
    // x and y direction
    Vector3<Scalar> feedback;
    for (uint32_t i = 0; i < feedback.size(); i++) {
      feedback(i) =
          params_.k_vel[i] *
          (velocity_error(i) +
           params_.k_pos[i] * utils::SatSmooth0(position_error(i), 1.0));
    }

    // const auto rotor_drag =
    //     computeDrag(refs.velocity, refs.acceleration, refs.yaw);

    // safety logic:
    // if int_flag && alti_threshold == true: enable integration
    // if int_flag == false : reset integration
    // if alti_threshold == false && int_flag == true: hold integration

    bool passAltThreshold = (state.position.z() > params_.de_height_threshold);
    // m * g * [0,0,1]
    Vector3<Scalar> vehicle_weight = -params_.vehicle_mass * kGravity;

    if (!intFlag) {
      disturbance_estimate_.setZero();
    } else if (intFlag && passAltThreshold) {
      // The expected thrust/acc
      // f = Rib * [0, 0, 1] * thrust
      Vector3<Scalar> expected_thrust =
          state.orientation * Vector3<Scalar>::UnitZ() * thrust_sp_;
      // The expected interial force: R_ib * [0,0,1] * a_b * m
      Vector3<Scalar> inertial_force =
          (state.orientation * state.acceleration - kGravity) *
          params_.vehicle_mass;
      // disturbance estimator
      disturbance_estimate_ -= params_.de_gain *
                               (disturbance_estimate_ + expected_thrust +
                                vehicle_weight - inertial_force) *
                               dt;
      // Bail on insane bounds
      if ((params_.de_lb.array() > params_.de_ub.array()).any()) {
        return {false, {Scalar(0), Quaternion<Scalar>::Identity()}, res};
      }
      Eigen::IOFormat a{
          Eigen::StreamPrecision, 0, ",", "\n;", "", "", "[", "]"};
      std::cout << "------------------------------\n";
      std::cout << std::setprecision(2) << std::fixed;
      std::cout << "z asix: " << state.orientation * Vector3<Scalar>::UnitZ()
                << '\n';
      // std::cout<<"expected thrust:
      // "<<expected_accel.transpose().format(a)<<'\n';
      std::cout << "expected thrust: " << expected_thrust.transpose().format(a)
                << '\n';
      std::cout << "disturbance_estimate_: "
                << disturbance_estimate_.transpose().format(a) << '\n';
      // std::cout<<"inertial_force:
      // "<<inertial_acc.transpose().format(a)<<'\n';
      std::cout << "inertial_force: " << inertial_force.transpose().format(a)
                << '\n';
      // Clamp disturbance estimate: TO DO: add saturation
      // disturbance_estimate_ =
      //     disturbance_estimate_.cwiseMax(params_.de_lb).cwiseMin(params_.de_ub);
    }

    std::cout << "dt: " << dt << '\n';
    std::cout << "intFlag is: " << std::boolalpha << intFlag
              << ", altiThreshold: " << std::boolalpha << passAltThreshold
              << '\n';

    // virtual control force: TO DO: check this function so that it take bounds
    // as arguments
    Vector3<Scalar> liftReq =
        -feedback - vehicle_weight - disturbance_estimate_;

    // attitude target
    Matrix3<Scalar> attitude_sp =
        details::accelerationVectorToRotation(liftReq, refs.yaw);

    // total required thrust
    thrust_sp_ = liftReq.dot(attitude_sp * Vector3Type::UnitZ());
    std::cout << "thrust setpoint (N) is: " << thrust_sp_ << '\n';
    // required thrust per rotor
    double thrustPerRotor =
        thrust_sp_ / static_cast<double>(params_.num_of_rotors);
    std::cout << "thrust per rotor (N) is: " << thrustPerRotor << '\n';
    res.accel_sp = liftReq;  // TO DO: need to change the name of this variable
    res.position_error = position_error;
    res.velocity_error = velocity_error;
    res.ude_effective = intFlag && passAltThreshold;
    res.ude_output = disturbance_estimate_;

    return {true, {thrustPerRotor, Quaternion<Scalar>(attitude_sp)}, res};
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
