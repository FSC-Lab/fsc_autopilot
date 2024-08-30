// Copyright © 2024 FSC Lab
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "fsc_autopilot/position_control/lqg_controller.hpp"

#include <iostream>

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/position_control/control.hpp"

namespace fsc {

ControlResult LQGController::run(const VehicleState& state,
                                 const Reference& refs, double dt,
                                 ContextBase* error) {
  using std::atan2;

  if (!params_valid_) {
    return {getFallBackSetpoint(), ControllerErrc::kInvalidParameters};
  }

  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;
  const auto& curr_acceleration = state.accel.linear;
  const auto& [ref_position, ref_orientation] = refs.state.pose;
  const auto& ref_velocity = refs.state.twist.linear;
  const auto& ref_yaw = refs.yaw;

  LQGControllerError* err = nullptr;
  if ((error != nullptr) && error->name() == "lqg_controller.error") {
    err = static_cast<decltype(err)>(error);
  }

  if (kf_reset_) {
    estimated_state_ << curr_position, curr_velocity;
    H_infinity_state_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }

  // double quat[4];
  // quat[0] = curr_orientation.w();
  // quat[1] = curr_orientation.x();
  // quat[2] = curr_orientation.y();
  // quat[3] = curr_orientation.z();

  // Eigen::Vector3d current_euler;
  // current_euler[0] =
  //     atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
  //           1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));  // roll
  // current_euler[1] =
  //     asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));  // pitch
  // current_euler[2] =
  //     atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
  //           1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));  // yaw

  Eigen::Matrix<double, 6, 1> combined_state;
  combined_state << curr_position, curr_velocity;

  // const Eigen::Vector3d raw_position_error = curr_position - ref_position;
  // const Eigen::Vector3d raw_velocity_error = curr_velocity - ref_velocity;

  const Eigen::Vector3d raw_position_error =
      estimated_state_.block<3, 1>(0, 0) - ref_position;
  const Eigen::Vector3d raw_velocity_error =
      estimated_state_.block<3, 1>(3, 0) - ref_velocity;

  Eigen::Vector3d position_error;
  Eigen::Vector3d velocity_error;

  if (apply_pos_err_saturation_) {
    position_error = SaturationSmoothing(raw_position_error, 1.0);
  } else {
    position_error = raw_position_error;
  }

  if (apply_vel_err_saturation_) {
    velocity_error = SaturationSmoothing(raw_velocity_error, 1.0);
  } else {
    velocity_error = raw_velocity_error;
  }

  //////////////////// LQR //////////////////////////
  Eigen::Matrix<double, 3, 6> gain_K;
  gain_K << -7.5, 0, 0, -5, 0, 0,  //
      0, -7.5, 0, 0, -5, 0,        //
      0, 0, -10, 0, 0, -7.5;       //

  Eigen::Matrix<double, 6, 1> combined_error;
  combined_error << position_error, velocity_error;

  Eigen::Matrix<double, 3, 1> feedback;
  feedback = -gain_K * combined_error;

  // std::cout << "thrust_setpoint_raw = " << thrust_setpoint_raw(0) << ", "
  //           << thrust_setpoint_raw(1) << ", " << thrust_setpoint_raw(2) <<
  //           "\n";

  // std::cout << "curr_position = " << curr_position(0) << ", "
  //           << curr_position(1) << ", " << curr_position(2) << "\n";

  // std::cout << "ref_position = " << ref_position(0) << ", " <<
  // ref_position(1)
  //           << ", " << ref_position(2) << "\n";

  ////////////////////// H_infinity ////////////////////
  Eigen::Matrix<double, 3, 12> F_star;
  F_star << -1.96134897, -1.96134897, -13.83873869, -0.74305035, -0.74305035,
      -3.18688477, -7.5, -7.5, -10., -5., -5., -7.5,  //
      -1.96134897, -1.96134897, -13.83873869, -0.74305035, -0.74305035,
      -3.18688477, -7.5, -7.5, -10., -5., -5., -7.5,  //
      -1.96134897, -1.96134897, -13.83873869, -0.74305035, -0.74305035,
      -3.18688477, -7.5, -7.5, -10., -5., -5., -7.5;  //

  Eigen::Matrix<double, 12, 12> Aq;
  Aq << 0., 0., 0., 1., 0., 0., -1., 0., 0., 0., 0., 0.,  //
      0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0., 0.,    //
      0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0.,    //
      -5.56549940, -1.15373469, -8.14043452, -3.37826491, -0.43708844,
      -1.87463810, 0., -4.41176471, -5.88235294, -1., -2.94117647,
      -4.41176471,  //
      -1.15373469, -5.56549940, -8.14043452, -0.43708844, -3.37826491,
      -1.87463810, -4.41176471, 0., -5.88235294, -2.94117647, -1.,
      -4.41176471,  //
      -1.15373469, -1.15373469, -14.0227875, -0.43708844, -0.43708844,
      -6.28640281, -4.41176471, -4.41176471, 0., -2.94117647, -2.94117647,
      -1.,                                                           //
      -8.88178420e-16, 0., 0., 0., 0., 0., -1., 0., 0., 1., 0., 0.,  //
      0., -8.88178420e-16, 0., 0., 0., 0., 0., -1., 0., 0., 1., 0.,  //
      0., 0., -3.55271368e-15, 0., 0., -4.44089210e-16, 0., 0., -1., 0., 0.,
      1.,                                                            //
      0., 0., 0., -2.22044605e-16, 0., 0., 0., 0., 0., -1., 0., 0.,  //
      0., 0., 0., 0., -2.22044605e-16, 0., 0., 0., 0., 0., -1., 0.,  //
      0., 0., -4.44089210e-16, 0., 0., -4.44089210e-16, 0., 0., 0., 0., 0.,
      -1.;  //

  Eigen::Matrix<double, 12, 6> Bq;
  Bq << -1., 0., 0., 0., 0., 0.,  //
      0., -1., 0., 0., 0., 0.,    //
      0., 0., -1., 0., 0., 0.,    //
      0., 0., 0., -1., 0., 0.,    //
      0., 0., 0., 0., -1., 0.,    //
      0., 0., 0., 0., 0., -1.,    //
      9., 0., 0., 0., 0., 0.,     //
      0., 9., 0., 0., 0., 0.,     //
      0., 0., 9., 0., 0., 0.,     //
      0., 0., 0., 9., 0., 0.,     //
      0., 0., 0., 0., 9., 0.,     //
      0., 0., 0., 0., 0., 9.;     //

  Eigen::Matrix<double, 6, 1> H_infinity_f = estimated_state_ - combined_state;

  H_infinity_state_ += (Aq * H_infinity_state_ + Bq * H_infinity_f) * dt;

  Eigen::Vector3d thrust_h_infinity = F_star * H_infinity_state_;

  //////////////////////// total thrust ////////////////////////
  const Eigen::Vector3d vehicle_weight = -vehicle_mass_ * kGravity;

  // without robust controller
  // Eigen::Vector3d thrust_setpoint_raw =
  //     -feedback - vehicle_weight + refs.thrust;

  // with robust controller
  Eigen::Vector3d thrust_setpoint_raw =
      -feedback - vehicle_weight + refs.thrust;

  std::cout << "feedback = " << -feedback(0) << ", " << -feedback(1) << ", "
            << -feedback(2) << "\n";
  std::cout << "thrust_h_infinity = " << thrust_h_infinity(0) << ", "
            << thrust_h_infinity(1) << ", " << thrust_h_infinity(2) << "\n";
  std::cout << "thrust_setpoint_raw = " << thrust_setpoint_raw(0) << ", "
            << thrust_setpoint_raw(1) << ", " << thrust_setpoint_raw(2)
            << "\n\n";

  Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, thrust_bnds_, max_tilt_angle_);

  Eigen::Vector3d thrust_kf = thrust_setpoint + vehicle_weight;

  ///////////////////////////// KF //////////////////////////
  Eigen::Matrix<double, 6, 6> gain_L;
  // gain_L << -1, 0.1, 0.1, 0.1, 0.1, 0.1,  //
  //     0.1, -1, 0.1, 0.1, 0.1, 0.1,        //
  //     0.1, 0.1, -1, 0.1, 0.1, 0.1,        //
  //     0.1, 0.1, 0.1, -1, 0.1, 0.1,        //
  //     0.1, 0.1, 0.1, 0.1, -1, 0.1,        //
  //     0.1, 0.1, 0.1, 0.1, 0.1, -1;        // x and y roughly -2

  gain_L << -10, 0, 0, 0, 0, 0,  //
      0, -10, 0, 0, 0, 0,        //
      0, 0, -10, 0, 0, 0,        //
      0, 0, 0, -10, 0, 0,        //
      0, 0, 0, 0, -10, 0,        //
      0, 0, 0, 0, 0, -10;        //

  Eigen::Matrix<double, 6, 6> matrix_A;
  matrix_A << 0, 0, 0, 1, 0, 0,  //
      0, 0, 0, 0, 1, 0,          //
      0, 0, 0, 0, 0, 1,          //
      0, 0, 0, 0, 0, 0,          //
      0, 0, 0, 0, 0, 0,          //
      0, 0, 0, 0, 0, 0;          //

  Eigen::Matrix<double, 6, 3> matrix_B;
  matrix_B << 0, 0, 0,          //
      0, 0, 0,                  //
      0, 0, 0,                  //
      1 / vehicle_mass_, 0, 0,  //
      0, 1 / vehicle_mass_, 0,  //
      0, 0, 1 / vehicle_mass_;  //

  Eigen::Matrix<double, 6, 1> estimated_state_dot;
  estimated_state_dot = matrix_A * estimated_state_ + matrix_B * thrust_kf +
                        gain_L * (estimated_state_ - combined_state);

  estimated_state_ += estimated_state_dot * dt;

  ////////////////////////////////////////////////
  // attitude target
  Eigen::Matrix3d attitude_sp =
      thrustVectorToRotation(thrust_setpoint, ref_yaw);

  double thrust = thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input = VehicleInput{thrust, Eigen::Quaterniond(attitude_sp)};
  if (err) {
    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;
    err->feedback = feedback;
    err->thrust_setpoint = thrust_setpoint;
    // msg output
    err->scalar_thrust_sp = thrust;
  }

  return result;
}

bool LQGController::setParams(const ParameterBase& params, LoggerBase& logger) {
  if (params.parameterFor() != "lqg_controller") {
    logger.log(Severity::kError, "Mismatch in parameter and receiver");

    return true;
  }

  if (!params.valid(logger)) {
    return false;
  }

  const auto& p = static_cast<const LQGControllerParameters&>(params);
  apply_pos_err_saturation_ = p.apply_pos_err_saturation;
  apply_vel_err_saturation_ = p.apply_vel_err_saturation;
  vehicle_mass_ = p.vehicle_mass;
  max_tilt_angle_ = p.max_tilt_angle;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  thrust_bnds_ = {p.min_thrust, p.max_thrust};

  params_valid_ = true;
  return true;
}

std::shared_ptr<ParameterBase> LQGController::getParams(
    bool use_default) const {
  if (use_default) {
    return std::make_shared<LQGControllerParameters>();
  }
  LQGControllerParameters params;

  params.apply_pos_err_saturation = apply_pos_err_saturation_;
  params.k_pos = k_pos_;
  params.apply_vel_err_saturation = apply_vel_err_saturation_;
  params.k_vel = k_vel_;
  params.min_thrust = thrust_bnds_.lower;
  params.max_thrust = thrust_bnds_.upper;
  params.max_tilt_angle = max_tilt_angle_;
  params.vehicle_mass = vehicle_mass_;

  return std::make_shared<LQGControllerParameters>(std::move(params));
}

void LQGController::toggleIntegration(bool value) { kf_reset_ = value; }

Eigen::Matrix<double, 6, 1> LQGController::getEstimatedState() const {
  return estimated_state_;
}

std::string LQGControllerParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "Quadrotor Mass: " << vehicle_mass << "\n"
      << "thrust bounds: [" << min_thrust << "," << max_thrust << "]\n"
      << "LQG Controller parameters:\nk_pos: "
      << k_pos.transpose().format(f)  //
      << "\nk_vel: " << k_vel.transpose().format(f);

  return oss.str();
}
bool LQGControllerParameters::valid(LoggerBase& logger) const {
  if (min_thrust >= max_thrust) {
    logger.log(Severity::kError,
               "`min_thrust` must be strictly less than `max_thrust`");
    return false;
  }

  if (vehicle_mass < 0.0) {
    logger.log(Severity::kError, "`vehicle_mass` must be positive");
    return false;
  }
  return true;
}

bool LQGControllerParameters::load(const ParameterLoaderBase& loader,
                                   LoggerBase& logger) {
  std::ignore =
      loader.getParam("apply_pos_err_saturation", apply_pos_err_saturation);
  k_pos <<  // Force a line break
      loader.param("k_pos/x", kDefaultKpXY),
      loader.param("k_pos/y", kDefaultKpXY),
      loader.param("k_pos/z", kDefaultKpZ);

  std::ignore =
      loader.getParam("apply_vel_err_saturation", apply_vel_err_saturation);
  k_vel <<  // Force a line break
      loader.param("k_vel/x", kDefaultKvXY),
      loader.param("k_vel/y", kDefaultKvXY),
      loader.param("k_vel/z", kDefaultKvZ);

  if (!loader.getParam("vehicle_mass", vehicle_mass)) {
    logger.log(Severity::kError, "Failed to load parameter `vehicle_mass`");
    return false;
  }

  std::ignore = loader.getParam("min_thrust", min_thrust);

  std::ignore = loader.getParam("max_thrust", max_thrust);

  std::ignore = loader.getParam("max_tilt_angle", max_tilt_angle);

  return true;
}

}  // namespace fsc
