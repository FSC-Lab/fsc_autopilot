#include "tracking_control/tracking_controller.hpp"

#include <iomanip>

#include "geometry_msgs/Vector3.h"
#include "tracking_control/control.hpp"
#include "tracking_control/controller_base.hpp"
#include "utils/utils.hpp"

namespace fsc {
TrackingController::TrackingController(ParametersSharedPtr params)
    : params_(std::move(params)) {}

ControlResult TrackingController::run(const VehicleState& state,
                                      const Reference& refs,
                                      ControlErrorBase* error) {
  using std::atan2;
  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;
  const auto& curr_acceleration = state.accel.linear;
  const auto& [ref_position, ref_orientation] = refs.state.pose;
  const auto& ref_velocity = refs.state.twist.linear;
  const auto& ref_yaw = refs.yaw;

  TrackingControllerError* err;
  if ((error != nullptr) && error->name() == "tracking_controller.error") {
    err = static_cast<decltype(err)>(error);
  }
  Eigen::Vector3d raw_position_error = curr_position - ref_position;
  Eigen::Vector3d raw_velocity_error = curr_velocity - ref_velocity;
  // Eigen::Vector3d
  const Eigen::Vector3d position_error =
      SaturationSmoothing(raw_position_error, 1.0);
  const Eigen::Vector3d velocity_error =
      SaturationSmoothing(raw_velocity_error, 1.0);
  // SaturationSmoothing(raw_velocity_error, 1.0);

  if (params_->vehicle_mass < 0.0) {
    return {false, getFallBackSetpoint()};
  }

  // bound the velocity and position error
  // kv * (ev + kp * sat(ep))
  // x and y direction
  const Eigen::Vector3d feedback = params_->k_vel.cwiseProduct(
      velocity_error + params_->k_pos.cwiseProduct(position_error));

  // safety logic:
  // if int_flag && alti_threshold == true: enable integration
  // if int_flag == false : reset integration
  // if alti_threshold == false && int_flag == true: hold integration

  bool pass_alt_threshold =
      (state.pose.position.z() > params_->de_height_threshold);
  // m * g * [0,0,1]
  Eigen::Vector3d vehicle_weight = -params_->vehicle_mass * kGravity;

  const bool int_flag = params_->ude_active;
  const double dt = params_->dt;

  Eigen::Vector3d expected_thrust;

  expected_thrust.setZero();

  if (!int_flag) {
    de_integral_.setZero();
  } else if (int_flag && pass_alt_threshold) {
    // The expected thrust/acc
    // f = Rib * [0, 0, 1] * thrust
    expected_thrust =
        state.pose.orientation * Eigen::Vector3d::UnitZ() * thrust_sp_;

    Eigen::Vector3d de_integrand;
    if (params_->ude_is_velocity_based) {
      de_integrand = disturbance_estimate_ + expected_thrust + vehicle_weight;
    } else {
      // The expected interial force: R_ib * [0,0,1] * a_b * m
      const Eigen::Vector3d inertial_force =
          (state.pose.orientation * state.accel.linear - kGravity) *
          params_->vehicle_mass;
      if (err) {
        err->inertialForce = inertial_force;
      }

      de_integrand = disturbance_estimate_ + expected_thrust + vehicle_weight -
                     inertial_force;
    }
    // disturbance estimator
    de_integral_ -= params_->de_gain * de_integrand * dt;
    // Bail on insane bounds
    if ((params_->de_lb.array() > params_->de_ub.array()).any()) {
      return {false, getFallBackSetpoint()};
    }

    Eigen::IOFormat a{Eigen::StreamPrecision, 0, ",", "\n;", "", "", "[", "]"};
    // std::cout << "------------------------------\n";
    // std::cout << std::setprecision(2) << std::fixed;
    // std::cout << "z asix: " << state.pose.orientation *
    // Eigen::Vector3d::UnitZ()
    //<< '\n';
    // std::cout<<"expected thrust:
    // "<<expected_accel.transpose().format(a)<<'\n';
    // std::cout << "expected thrust: " << expected_thrust.transpose().format(a)
    //<< '\n';
    // std::cout << "disturbance_estimate_: "
    //<< disturbance_estimate_.transpose().format(a) << '\n';
    // std::cout<<"inertial_force:
    // "<<inertial_acc.transpose().format(a)<<'\n';
    // std::cout << "inertial_force: " << inertial_force.transpose().format(a)
    //<< '\n';
    // Clamp disturbance estimate: TO DO: add saturation
    // disturbance_estimate_ =
    //     disturbance_estimate_.cwiseMax(params_->de_lb).cwiseMin(params_->de_ub);
  }

  // std::cout << "dt: " << dt << '\n';
  // std::cout << "intFlag is: " << std::boolalpha << int_flag
  //<< ", altiThreshold: " << std::boolalpha << pass_alt_threshold
  //<< '\n';
  //
  if (params_->ude_is_velocity_based) {
    const Eigen::Vector3d de_damping =
        params_->de_gain * params_->vehicle_mass * curr_velocity;
    disturbance_estimate_ = de_integral_ + de_damping;
  } else {
    disturbance_estimate_ = de_integral_;
  }

  // virtual control force: TO DO: check this function so that it take bounds
  // as arguments
  Eigen::Vector3d lift_req_raw =
      -feedback - vehicle_weight - disturbance_estimate_;
  Eigen::Vector3d lift_req = MultirotorThrustLimiting(
      lift_req_raw, {params_->min_z_accel, params_->max_z_accel},
      params_->max_tilt_angle);

  // attitude target
  Eigen::Matrix3d attitude_sp = thrustVectorToRotation(lift_req, ref_yaw);

  // total required thrust
  thrust_sp_ = lift_req.dot(attitude_sp * Eigen::Vector3d::UnitZ());
  // std::cout << "thrust setpoint (N) is: " << thrust_sp_ << '\n';
  //  required thrust per rotor
  double thrust_per_rotor =
      thrust_sp_ / static_cast<double>(params_->num_of_rotors);
  // std::cout << "thrust per rotor (N) is: " << thrust_per_rotor << '\n';

  ControlResult result;
  result.success = true;
  result.setpoint.input.thrust = thrust_per_rotor;
  result.setpoint.input.command = Eigen::Quaterniond(attitude_sp);
  if (err) {
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->accel_sp =
        lift_req;  // TO DO: need to change the name of this variable
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->ude_effective = int_flag && pass_alt_threshold;
    err->ude_output = disturbance_estimate_;
    // msg output
    err->altiThreshold = pass_alt_threshold;
    err->intFlag = int_flag;
    err->thrust_sp = thrust_sp_;
    err->thrustPerRotor =
        thrust_sp_ / static_cast<double>(params_->num_of_rotors);
    err->expectedThrust = expected_thrust;
    err->disturbanceEstimate = de_integral_;
  }

  return result;
}
}  // namespace fsc
