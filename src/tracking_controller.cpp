#include "tracking_control/tracking_controller.hpp"

#include <iomanip>

#include "tracking_control/control.hpp"
#include "tracking_control/controller_base.hpp"

namespace fsc {
TrackingController::TrackingController(TrackingControllerParameters params)
    : params_(std::move(params)) {}

ControlResult TrackingController::run(const VehicleState& state,
                                      const Setpoint& refs, double dt) {
  using std::atan2;
  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;
  const auto& curr_acceleration = state.accel.linear;
  const auto& [ref_position, ref_orientation] = refs.state.pose;
  const auto& ref_velocity = refs.state.twist.linear;
  const auto& ref_yaw = QuaternionToEulerAngles(ref_orientation).z();

  auto error = std::make_shared<TrackingControllerError>();
  Eigen::Vector3d position_error = curr_position - ref_position;
  Eigen::Vector3d velocity_error = curr_velocity - ref_velocity;

  error->position_error = std::move(position_error);
  error->velocity_error = std::move(velocity_error);
  if (params_.vehicle_mass < 0.0) {
    return {false, getFallBackSetpoint(), std::move(error), "Mass is negative"};
  }

  // bound the velocity and position error
  // kv * (ev + kp * sat(ep))
  // x and y direction
  Eigen::Vector3d feedback;
  for (uint32_t i = 0; i < feedback.size(); i++) {
    feedback(i) =
        params_.k_vel[i] *
        (velocity_error(i) +
         params_.k_pos[i] * utils::SatSmooth0(position_error(i), 1.0));
  }

  // safety logic:
  // if int_flag && alti_threshold == true: enable integration
  // if int_flag == false : reset integration
  // if alti_threshold == false && int_flag == true: hold integration

  bool pass_alt_threshold =
      (state.pose.position.z() > params_.de_height_threshold);
  // m * g * [0,0,1]
  Eigen::Vector3d vehicle_weight = -params_.vehicle_mass * kGravity;

  bool int_flag;
  if (!state.ctx->getFlag("interrupt_ude", int_flag)) {
    return {false, getFallBackSetpoint(), std::move(error),
            "Failed to get UDE interrupt"};
  }
  if (!int_flag) {
    disturbance_estimate_.setZero();
  } else if (int_flag && pass_alt_threshold) {
    // The expected thrust/acc
    // f = Rib * [0, 0, 1] * thrust
    Eigen::Vector3d expected_thrust =
        state.pose.orientation * Eigen::Vector3d::UnitZ() * thrust_sp_;
    // The expected interial force: R_ib * [0,0,1] * a_b * m
    Eigen::Vector3d inertial_force =
        (state.pose.orientation * state.accel.linear - kGravity) *
        params_.vehicle_mass;
    // disturbance estimator
    disturbance_estimate_ -= params_.de_gain *
                             (disturbance_estimate_ + expected_thrust +
                              vehicle_weight - inertial_force) *
                             dt;
    // Bail on insane bounds
    if ((params_.de_lb.array() > params_.de_ub.array()).any()) {
      return {false, getFallBackSetpoint(), std::move(error)};
    }

    Eigen::IOFormat a{Eigen::StreamPrecision, 0, ",", "\n;", "", "", "[", "]"};
    std::cout << "------------------------------\n";
    std::cout << std::setprecision(2) << std::fixed;
    std::cout << "z asix: " << state.pose.orientation * Eigen::Vector3d::UnitZ()
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
  std::cout << "intFlag is: " << std::boolalpha << int_flag
            << ", altiThreshold: " << std::boolalpha << pass_alt_threshold
            << '\n';

  // virtual control force: TO DO: check this function so that it take bounds
  // as arguments
  Eigen::Vector3d lift_req = MultirotorThrustLimiting(
      -feedback - vehicle_weight - disturbance_estimate_,
      {params_.min_z_accel, params_.max_z_accel}, params_.max_tilt_angle);

  // attitude target
  Eigen::Matrix3d attitude_sp = thrustVectorToRotation(lift_req, ref_yaw);

  // total required thrust
  thrust_sp_ = lift_req.dot(attitude_sp * Eigen::Vector3d::UnitZ());
  std::cout << "thrust setpoint (N) is: " << thrust_sp_ << '\n';
  // required thrust per rotor
  double thrust_per_rotor =
      thrust_sp_ / static_cast<double>(params_.num_of_rotors);
  std::cout << "thrust per rotor (N) is: " << thrust_per_rotor << '\n';

  ControlResult result;
  result.success = true;
  result.setpoint.input.thrust = thrust_per_rotor;
  result.setpoint.input.command = Eigen::Quaterniond(attitude_sp);

  error->accel_sp =
      lift_req;  // TO DO: need to change the name of this variable
  error->position_error = position_error;
  error->velocity_error = velocity_error;
  error->ude_effective = int_flag && pass_alt_threshold;
  error->ude_output = disturbance_estimate_;
  result.error = std::move(error);

  return result;
}
}  // namespace fsc
