#include "fsc_autopilot/position_control/lqg_controller.hpp"

#include <iostream>

#include "fsc_autopilot/ude/ude_factory.hpp"

namespace fsc {
ControlResult LQGController::run(const VehicleState& state,
                                 const Reference& refs, double dt,
                                 ContextBase* error) {
  using std::atan2;

  // Check if parameters are valid

  if (!params_valid_) {
    return {getFallBackSetpoint(), ControllerErrc::kInvalidParameters};
  }

  // Extract current state and reference values

  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;
  const auto& curr_acceleration = state.accel.linear;
  const auto& [ref_position, ref_orientation] = refs.state.pose;
  const auto& ref_velocity = refs.state.twist.linear;
  const auto& ref_yaw = refs.yaw;

  // get current euler angles from current quaternion
  double quat[4];
  quat[0] = curr_orientation.w();
  quat[1] = curr_orientation.x();
  quat[2] = curr_orientation.y();
  quat[3] = curr_orientation.z();

  Eigen::Vector3d current_euler;
  current_euler[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                           1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
  current_euler[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
  current_euler[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                           1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));

  // define current state x
  LQGKF::State x;
  x << curr_position, curr_velocity, current_euler[2];

  // initialize estimated state
  if (!lqg_active_) {
    // std::cout << "x_hat initialized" << "\n\n";
    x_hat = x;
  }

  std::cout << "x_hat = " << x_hat << "\n\n";

  // Initialize error pointer
  TrackingControllerError* err = nullptr;
  if ((error != nullptr) && error->name() == "tracking_controller.error") {
    err = static_cast<decltype(err)>(error);
  }

  Eigen::Vector3d estimated_position;
  Eigen::Vector3d estimated_velocity;
  estimated_position << x_hat[0], x_hat[1], x_hat[2];
  estimated_velocity << x_hat[3], x_hat[4], x_hat[5];
  double estimated_yaw = x_hat[6];

  // Calculate raw position and velocity errors
  // const Eigen::Vector3d raw_position_error = curr_position - ref_position;
  // const Eigen::Vector3d raw_velocity_error = curr_velocity - ref_velocity;

  const Eigen::Vector3d raw_position_error = estimated_position - ref_position;
  const Eigen::Vector3d raw_velocity_error = estimated_velocity - ref_velocity;

  // rotate the error from world frame to body frame
  Eigen::Matrix<double, 3, 3> R_b_i;
  R_b_i = yawToRotation(ref_yaw);

  const Eigen::Vector3d position_error_body_frame = R_b_i * raw_position_error;
  const Eigen::Vector3d velocity_error_body_frame = R_b_i * raw_velocity_error;

  // Apply position and velocity error saturation if enabled
  Eigen::Vector3d position_error;
  Eigen::Vector3d velocity_error;

  if (apply_pos_err_saturation_) {
    position_error = SaturationSmoothing(position_error_body_frame, 1.0);
  } else {
    position_error = position_error_body_frame;
  }

  if (apply_vel_err_saturation_) {
    velocity_error = SaturationSmoothing(velocity_error_body_frame, 1.0);
  } else {
    velocity_error = velocity_error_body_frame;
  }

  // Calculate vehicle weight m * g * [0,0,1]
  const Eigen::Vector3d vehicle_weight = -vehicle_mass_ * kGravity;

  ////////////////////// LQG controller /////////////////////////
  // Step 1: Initialize the 4x7 matrix nominal control gain
  Eigen::Matrix<double, 4, 7> gain_K;
  // gain_K << 1, 0, 0, 1.1865, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0,
  // -1, 0, 0, 0, 0, 0, 0, 0, -1;
  // original setup with eye(7), z-axis reaction slow, pitch and roll too
  // aggressive

  // gain_K << 0.1, 0, 0, 0.2253, 0, 0, 0, 0, -0.1, 0, 0, -0.1, 0, 0, 0, 0, -1,
  // 0,
  //     0, -1, 0, 0, 0, 0, 0, 0, 0, -0.1;
  // decreased row 1, 2, 4, 5, 7 of eye(7) to 0.1
  // x and y much better, z-axis still reacting slow

  gain_K << 0.25, 0, 0, 0.4055, 0, 0, 0, 0, -0.25, 0, 0, -0.25, 0, 0, 0, 0, -10,
      0, 0, -10, 0, 0, 0, 0, 0, 0, 0, -1;
  // increased row 3, 6 of eye(7) to 10

  // Step 2: Initialize the 7x1 matrix combined_error-feedback -
  // vehicle_weight
  // - ude_value
  LQGKF::State combined_error;
  combined_error << position_error(0, 0), position_error(1, 0),
      position_error(2, 0), velocity_error, ref_yaw;

  // Step 3: Perform the matrix multiplication to get matrix control_input_u
  Eigen::Matrix<double, 4, 1> control_input_u = -gain_K * combined_error;

  double roll_saturated = std::min(control_input_u(1, 0), 0.75);
  double pitch_saturated = std::min(control_input_u(0, 0), 0.75);

  // update KF

  const Eigen::Vector3d position_body_frame = R_b_i * curr_position;
  const Eigen::Vector3d velocity_body_frame = R_b_i * curr_velocity;

  LQGKF::State x_body_frame;

  x_body_frame << position_body_frame, velocity_body_frame, ref_yaw;

  Eigen::Matrix<double, 4, 1> kf_control_input_u;
  kf_control_input_u << pitch_saturated, roll_saturated, control_input_u(3, 0),
      0;

  LQGKF::State x_hat_body = kf_.updateKF(x_body_frame, kf_control_input_u, dt);

  Eigen::Vector3d position_hat_body;
  Eigen::Vector3d velocity_hat_body;
  position_hat_body << x_hat_body[0], x_hat_body[1], x_hat_body[2];
  velocity_hat_body << x_hat_body[3], x_hat_body[4], x_hat_body[5];

  // rotate the error from body frame to world frame
  Eigen::Matrix<double, 3, 3> R_i_b;
  R_i_b = bodyToWorld(ref_yaw);

  const Eigen::Vector3d position_hat = R_i_b * position_hat_body;
  const Eigen::Vector3d velocity_hat = R_i_b * velocity_hat_body;

  x_hat << position_hat, velocity_hat, ref_yaw;

  // std::cout << "x_hat = " << x_hat << "\n\n";
  std::cout << "x = " << x << "\n\n";

  Eigen::Vector3d euler_angles;
  euler_angles << roll_saturated, pitch_saturated, ref_yaw;

  Eigen::Matrix3d attitude_sp;

  attitude_sp = eulerAnglesToRotationMatrix(euler_angles);  // roll pitch yaw

  // Compute raw thrust setpoint
  Eigen::Vector3d thrust_setpoint_raw =
      control_input_u(2, 0) * Eigen::Vector3d::UnitZ() - vehicle_weight;

  // Apply thrust limiting to get final thrust setpoint
  Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, thrust_bnds_, max_tilt_angle_);

  // Calculate total required thrust
  scalar_thrust_setpoint_ =
      thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());

  double thrust_per_rotor =
      scalar_thrust_setpoint_ / static_cast<double>(num_rotors_);

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input =
      VehicleInput{thrust_per_rotor, Eigen::Quaterniond(attitude_sp)};

  // If there is an error pointer, populate error fields
  if (err) {
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->thrust_setpoint = thrust_setpoint;
    // msg output
    err->scalar_thrust_sp = scalar_thrust_setpoint_;
    err->thrust_per_rotor =
        scalar_thrust_setpoint_ / static_cast<double>(num_rotors_);
  }

  std::cout << "XSY controller is working \n\n";

  return result;
}

bool LQGController::setParams(const ParameterBase& params, LoggerBase* logger) {
  if (params.parameterFor() != "tracking_controller") {
    LOG_OPTIONAL(logger, Severity::kError,
                 "Mismatch in parameter and receiver");

    return true;
  }

  if (!params.valid()) {
    LOG_OPTIONAL(logger, Severity::kError, "Parameters are invalid");
    return false;
  }

  const auto& p = static_cast<const LQGControllerParameters&>(params);
  num_rotors_ = p.num_of_rotors;
  apply_pos_err_saturation_ = p.apply_pos_err_saturation;
  apply_vel_err_saturation_ = p.apply_vel_err_saturation;
  vehicle_mass_ = p.vehicle_mass;
  max_tilt_angle_ = p.max_tilt_angle;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  thrust_bnds_ = {p.min_thrust, p.max_thrust};
  // TODO(Shangyi): set initial x_hat
  kf_.initialize(p.vehicle_mass, {});

  if (!ude_) {
    auto ude = UDEFactory::Create(p.ude_type, logger);
    if (!ude) {
      return false;
    }
    ude_ = std::move(ude);
  }
  if (!ude_->setParams(p.ude_params)) {
    LOG_OPTIONAL(logger, Severity::kError, "Failed to set UDE parameters");
    return false;
  }

  params_valid_ = true;
  return true;
}

LQGKF::LQGKF(double mass, const State& x_hat_init)
    : mass_(mass), x_hat_(x_hat_init) {
  A_ << 0, 0, 0, 1, 0, 0, 0,  //
      0, 0, 0, 0, 1, 0, 0,    //
      0, 0, 0, 0, 0, 1, 0,    //
      0, 0, 0, 0, 0, 0, 0,    //
      0, 0, 0, 0, 0, 0, 0,    //
      0, 0, 0, 0, 0, 0, 0,    //
      0, 0, 0, 0, 0, 0, 0;    //

  B_ << 0, 0, 0, 0,       //
      0, 0, 0, 0,         //
      0, 0, 0, 0,         //
      -9.81, 0, 0, 0,     //
      0, 9.81, 0, 0,      //
      0, 0, 1 / mass, 0,  //
      0, 0, 0, 1;         //

  L_ << -0.25, 0, 0, 0, 0, 0, 0,  //
      0, -0.25, 0, 0, 0, 0, 0,    //
      0, 0, -10, 0, 0, 0, 0,      //
      0, 0, 0, -0.25, 0, 0, 0,    //
      0, 0, 0, 0, -0.25, 0, 0,    //
      0, 0, 0, 0, 0, -10, 0,      //
      0, 0, 0, 0, 0, 0, -1;       //
}
LQGKF::State LQGKF::updateKF(const State& x, const Input& control_input_u,
                             double dt) {
  // std::cout << "ude_active_ = " << ude_active_ << "\n";
  // std::cout << "lqg_active_ = " << lqg_active_ << "\n";

  // Kalman Filter matrices
  Eigen::VectorXd x_hat_dot =
      A_ * x_hat_ + B_ * control_input_u + L_ * (x_hat_ - x);

  const bool int_flag = lqg_active_;
  if (!int_flag) {
    x_hat_ = x;
  } else {
    // Kalman Filter
    x_hat_ += x_hat_dot * dt;
  }

  // Optionally clamp x_hat within certain bounds if needed
  // x_hat = x_hat.cwiseMax(lower_bound).cwiseMin(upper_bound);

  return x_hat_;
}
bool LQGKF::initialize(double mass, const State& x_hat_init) {
  mass_ = mass;
  x_hat_ = x_hat_init;
  return x_hat_.allFinite() && mass_ > 0.0;
}
}  // namespace fsc
