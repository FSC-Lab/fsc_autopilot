#include "tracking_control/tracking_controller.hpp"

#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"
#include "tracking_control/multirotor_acceleration_limiter.hpp"

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

  ControlResult result;
  auto error = std::make_shared<TrackingControllerError>();
  Eigen::Vector3d position_error = curr_position - ref_position;
  Eigen::Vector3d velocity_error = curr_velocity - ref_velocity;
  // auto position_error = state.position - ref_position;
  // auto velocity_error = state.velocity - ref_velocity;

  error->position_error = std::move(position_error);
  error->velocity_error = std::move(velocity_error);
  if (params_.vehicle_mass < 0.0) {
    result.success = false;
    result.setpoint.input.command = Eigen::Quaterniond::Identity();
    result.setpoint.input.thrust = 0.0;
    result.error = std::move(error);
    return result;
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

  bool ude_effective = (curr_position.z() > params_.de_height_threshold);

  if (ude_effective) {
    // The expected acceleration: R_ib * [0,0,1] * a_b
    Eigen::Vector3d expected_accel =
        curr_orientation * Eigen::Vector3d::UnitZ() * thrust_sp_;
    // m * g * [0,0,1]
    Eigen::Vector3d vehicle_weight = -params_.vehicle_mass * kGravity;
    // R_ib * a_b * m
    Eigen::Vector3d inertial_acc =
        curr_orientation * curr_acceleration - kGravity;
    // disturbance estimator
    disturbance_estimate_ -=
        params_.de_gain *
        (disturbance_estimate_ + expected_accel - kGravity - inertial_acc) * dt;

    // Eigen::IOFormat a{Eigen::StreamPrecision, 0, ",", "\n;", "", "", "[",
    // "]"}; std::cout<<"------------------------------\n"; std::cout <<
    // std::setprecision(2) << std::fixed; std::cout<<"expected thrust:
    // "<<expected_accel.transpose().format(a)<<'\n';
    // std::cout<<"disturbance_estimate_:
    // "<<disturbance_estimate_.transpose().format(a)<<'\n';
    // std::cout<<"inertial_force:
    // "<<inertial_acc.transpose().format(a)<<'\n'; std::cout<<"dt:
    // "<<dt<<'\n';

    // Bail on insane bounds
    if ((params_.de_lb.array() > params_.de_ub.array()).any()) {
      result.success = false;
      result.setpoint.input.command = Eigen::Quaterniond::Identity();
      result.setpoint.input.thrust = 0.0;
      result.error = std::move(error);

      return result;
    }
    // Clamp disturbance estimate
    // disturbance_estimate_ =
    //     disturbance_estimate_.cwiseMax(params_.de_lb).cwiseMin(params_.de_ub);
  } else {
    disturbance_estimate_.setZero();
  }

  const auto accel_sp = MultirotorAccelerationLimiting(
      -feedback + kGravity - disturbance_estimate_,
      {params_.min_z_accel, params_.max_z_accel}, params_.max_tilt_angle);

  auto attitude_sp = accelerationVectorToRotation(accel_sp, ref_yaw);

  thrust_sp_ = accel_sp.dot(attitude_sp * Eigen::Vector3d::UnitZ());

  result.success = true;
  result.setpoint.input.thrust = thrust_sp_;
  result.setpoint.input.command = Eigen::Quaterniond(attitude_sp);

  error->accel_sp = accel_sp;
  error->position_error = position_error;
  error->velocity_error = velocity_error;
  error->ude_effective = ude_effective;
  error->ude_output = disturbance_estimate_;
  result.error = std::move(error);

  return result;
}
}  // namespace fsc
