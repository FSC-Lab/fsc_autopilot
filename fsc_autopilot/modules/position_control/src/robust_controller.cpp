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

#include "fsc_autopilot/position_control/robust_controller.hpp"

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/math/numbers.hpp"
#include "fsc_autopilot/position_control/control.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"
#include "fsc_autopilot/position_control/position_controller_factory.hpp"

namespace fsc {

PositionControlResult RobustController::run(
    const VehicleState& state, const PositionControllerReference& refs,
    double dt, ContextBase* error) {
  // * Implement checks INDEPENDENTLY from ParameterBase::valid, with an eye
  // * towards the minimum that doesn't break this controller
  if (!thrust_bnds_.valid() || vehicle_mass_ < 0.0 ||
      (k_pos_.array() < 0.0).any() || (k_vel_.array() < 0.0).any()) {
    return {{}, ControllerErrc::kInvalidParameters};
  }

  const auto& curr_position = state.pose.position;
  const auto& curr_velocity = state.twist.linear;
  const auto& [ref_position, ref_velocity, ref_accel, ref_thrust, ref_yaw] =
      refs;

  const Eigen::Vector3d raw_position_error = curr_position - ref_position;
  const Eigen::Vector3d raw_velocity_error = curr_velocity - ref_velocity;

  Eigen::Vector3d position_error;
  Eigen::Vector3d velocity_error;

  // Velocity and position errors are bounded
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

  Eigen::Vector3d pd_term;
  for (int i = 0; i < 3; ++i) {
    pd_term[i] = ProportionalDerivative(position_error[i], velocity_error[i],
                                        k_pos_[i], k_vel_[i], dt, form_);
  }

  // m * g * [0,0,1]
  const Eigen::Vector3d vehicle_weight =
      -vehicle_mass_ * Eigen::Vector3d::UnitZ() * numbers::std_gravity;

  // Our controller operates in the thrust world
  Eigen::Vector3d feedforward = ref_thrust;

  if (!ref_accel.isZero()) {  // Avoid the extra ops if possible
    feedforward += ref_accel * numbers::std_gravity;
  }

  const Eigen::Vector3d thrust_setpoint_raw =
      -pd_term - vehicle_weight + feedforward;
  const Eigen::Vector3d thrust_setpoint = MultirotorThrustLimiting(
      thrust_setpoint_raw, thrust_bnds_, max_tilt_angle_);

  // attitude target
  Eigen::Matrix3d attitude_sp =
      thrustVectorToRotation(thrust_setpoint, ref_yaw);

  double thrust = thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());

  PositionControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.input = {thrust, Eigen::Quaterniond(attitude_sp)};
  if (PositionControllerState* err = nullptr;
      (error != nullptr) && error->name() == "position_controller_state") {
    err = static_cast<decltype(err)>(error);
    err->position_reference = ref_position;
    err->velocity_reference = ref_velocity;

    err->position = curr_position;
    err->velocity = curr_velocity;
    err->acceleration = state.accel.linear;

    err->position_error = raw_position_error;
    err->velocity_error = raw_velocity_error;

    err->output = thrust_setpoint;
  }
  return result;
}

bool RobustController::setParams(const ParameterBase& params,
                                 LoggerBase& logger) {
  if (params.parameterFor() != "robust_controller") {
    logger.log(Severity::kError, "Mismatch in parameter and receiver");

    return true;
  }

  if (!params.valid(logger)) {
    return false;
  }

  const auto& p = static_cast<const RobustControllerParameters&>(params);
  apply_pos_err_saturation_ = p.apply_pos_err_saturation;
  apply_vel_err_saturation_ = p.apply_vel_err_saturation;
  vehicle_mass_ = p.vehicle_mass;
  max_tilt_angle_ = p.max_tilt_angle;
  form_ = p.form;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  thrust_bnds_ = {p.min_thrust, p.max_thrust};

  return true;
}

std::shared_ptr<ParameterBase> RobustController::getParams(
    bool use_default) const {
  if (use_default) {
    return std::make_shared<RobustControllerParameters>();
  }
  RobustControllerParameters params;

  params.apply_pos_err_saturation = apply_pos_err_saturation_;
  params.apply_vel_err_saturation = apply_vel_err_saturation_;
  params.form = form_;
  params.k_pos = k_pos_;
  params.k_vel = k_vel_;
  params.min_thrust = thrust_bnds_.lower;
  params.max_thrust = thrust_bnds_.upper;
  params.max_tilt_angle = max_tilt_angle_;
  params.vehicle_mass = vehicle_mass_;

  return std::make_shared<RobustControllerParameters>(std::move(params));
}

std::string RobustControllerParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "Quadrotor Mass: " << vehicle_mass << "\n"
      << "thrust bounds: [" << min_thrust << "," << max_thrust << "]\n"
      << "robust Controller parameters:\nform: " << to_string(form)
      << "\nk_pos: " << k_pos.transpose().format(f)  //
      << "\nk_vel: " << k_vel.transpose().format(f);

  return oss.str();
}
bool RobustControllerParameters::valid(LoggerBase& logger) const {
  if (min_thrust >= max_thrust) {
    logger.log(Severity::kError,
               "`min_thrust` must be strictly less than `max_thrust`");
    return false;
  }

  if (vehicle_mass < 0.0) {
    logger.log(Severity::kError, "`vehicle_mass` must be positive");
    return false;
  }

  if ((k_pos.array() < 0.0).any()) {
    logger.log(Severity::kError, "`k_pos` must be all positive");
    return false;
  }
  if ((k_vel.array() < 0.0).any()) {
    logger.log(Severity::kError, "`k_pos` must be all positive");
    return false;
  }

  return true;
}

bool RobustControllerParameters::load(const ParameterLoaderBase& loader,
                                      LoggerBase& logger) {
  using namespace std::string_literals;  // NOLINT
  std::ignore =
      loader.getParam("apply_pos_err_saturation", apply_pos_err_saturation);
  std::ignore =
      loader.getParam("apply_vel_err_saturation", apply_vel_err_saturation);

  const auto form_str = loader.param("form", "nested"s);
  if (form_str == "ideal") {
    form = PIDForm::kIdeal;
  } else if (form_str == "nested") {
    form = PIDForm::kNested;
  } else if (form_str == "parallel") {
    form = PIDForm::kParallel;
  } else {
    logger.log(Severity::kError)
        << "Invalid PD controller form `" << form_str << "`";
    return false;
  }

  k_pos <<  // Force a line break
      loader.param("k_pos/x", kDefaultKpXY),
      loader.param("k_pos/y", kDefaultKpXY),
      loader.param("k_pos/z", kDefaultKpZ);

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

REGISTER_POSITION_CONTROLLER(RobustController, "robust_controller");

}  // namespace fsc
