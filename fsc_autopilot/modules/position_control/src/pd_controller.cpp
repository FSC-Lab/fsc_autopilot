// Copyright © 2024 yourname
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

#include "fsc_autopilot/position_control/pd_controller.hpp"

#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/math/numbers.hpp"
#include "fsc_autopilot/position_control/control.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"
#include "fsc_autopilot/position_control/position_controller_factory.hpp"

namespace fsc {

PositionControlResult PDController::run(const VehicleState& state,
                                        const PositionControllerReference& refs,
                                        double dt, ContextBase* error) {
  if (vehicle_mass_ < 0.0 || (k_pos_.array() < 0.0).any() ||
      (k_vel_.array() < 0.0).any()) {
    return {{}, ControllerErrc::kInvalidParameters};
  }
  const auto& [curr_position, curr_orientation] = state.pose;
  const auto& curr_velocity = state.twist.linear;

  const auto& [ref_position, ref_velocity, ref_accel, ref_thrust, ref_yaw] =
      refs;

  // * Velocity and position errors are NOT bounded since those weren't used in
  // * either the papers or mavros_controllers
  const Eigen::Vector3d position_error = curr_position - ref_position;
  const Eigen::Vector3d velocity_error = curr_velocity - ref_velocity;

  Eigen::Vector3d pd_term;
  for (int i = 0; i < 3; ++i) {
    pd_term[i] = ProportionalDerivative(position_error[i], velocity_error[i],
                                        k_pos_[i], k_vel_[i], dt, form_);
  }

  const Eigen::Vector3d vehicle_weight =
      -vehicle_mass_ * Eigen::Vector3d::UnitZ() * numbers::std_gravity;

  // Our controller operates in the thrust world
  Eigen::Vector3d feedforward = ref_thrust;

  if (!ref_accel.isZero()) {  // Avoid the extra ops if possible
    feedforward += ref_accel * numbers::std_gravity;
  }

  // NOLINTBEGIN(readability-identifier-naming)
  const Eigen::Matrix3d R = curr_orientation.toRotationMatrix();
  const Eigen::Vector3d drag = R * D_ * R.transpose() * curr_velocity;
  // NOLINTEND(readability-identifier-naming)

  const Eigen::Vector3d thrust_setpoint =
      -pd_term - vehicle_weight - drag + feedforward;

  // * NO thrust or tilt limiting since those weren't called for in either the
  // * papers or mavros_controllers
  const Eigen::Matrix3d attitude_sp =
      thrustVectorToRotation(thrust_setpoint, ref_yaw);

  const double thrust =
      thrust_setpoint.dot(attitude_sp * Eigen::Vector3d::UnitZ());

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

    err->position_error = position_error;
    err->velocity_error = velocity_error;

    err->output = thrust_setpoint;
  }

  return result;
}

bool PDController::setParams(const ParameterBase& params, LoggerBase& logger) {
  if (params.parameterFor() != "pd_controller") {
    logger.log(Severity::kError, "Mismatch in parameter and receiver");
  }

  if (!params.valid(logger)) {
    return false;
  }
  const auto& p = static_cast<const PDControllerParameters&>(params);
  vehicle_mass_ = p.vehicle_mass;
  k_pos_ = p.k_pos;
  k_vel_ = p.k_vel;
  D_ = p.drag_coeffs.asDiagonal();
  return true;
}

std::shared_ptr<ParameterBase> PDController::getParams(bool use_default) const {
  if (use_default) {
    return std::make_shared<PDControllerParameters>();
  }
  PDControllerParameters params;

  params.vehicle_mass = vehicle_mass_;
  params.k_pos = k_pos_;
  params.k_vel = k_vel_;
  params.drag_coeffs = D_.diagonal();

  return std::make_shared<PDControllerParameters>(std::move(params));
}

bool PDControllerParameters::valid(LoggerBase& logger) const {
  if (vehicle_mass < 0.0) {
    logger.log(Severity::kError, "`vehicle_mass` must be positive");
    return false;
  }
  return true;
}
bool PDControllerParameters::load(const ParameterLoaderBase& loader,
                                  LoggerBase& logger) {
  using namespace std::string_literals;  // NOLINT
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

  drag_coeffs << loader.param("drag_coeffs/x", 1.0),
      loader.param("drag_coeffs/y", 1.0), loader.param("drag_coeffs/z", 1.0);

  return true;
}

std::string PDControllerParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "Quadrotor Mass: " << vehicle_mass << "\n"
      << "PD Controller parameters:\nform: " << to_string(form)
      << "\nk_pos: " << k_pos.transpose().format(f)  //
      << "\nk_vel: " << k_vel.transpose().format(f)
      << "\ndrag_coeffs: " << drag_coeffs.transpose().format(f);

  return oss.str();
}

REGISTER_POSITION_CONTROLLER(PDController, "pd_controller");
}  // namespace fsc
