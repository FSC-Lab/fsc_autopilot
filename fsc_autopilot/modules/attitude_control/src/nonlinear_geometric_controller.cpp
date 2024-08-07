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

#include "fsc_autopilot/attitude_control/nonlinear_geometric_controller.hpp"

#include <memory>
#include <utility>

#include "fsc_autopilot/attitude_control/attitude_controller_factory.hpp"
#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/math/rotation.hpp"

namespace fsc {

AttitudeControlResult NonlinearGeometricController::run(
    const VehicleState& state, const AttitudeReference& refs, double dt,
    [[maybe_unused]] ContextBase* error) {
  const auto rotmat = state.pose.orientation.toRotationMatrix();
  const auto rotmat_sp = refs.orientation.toRotationMatrix();

  AttitudeControllerState* err;
  if ((error != nullptr) && error->name() == "attitude_controller_state") {
    err = static_cast<decltype(err)>(error);
  }
  // e_r = 1 / 2 * (Rd.T * R - R.T * Rd)
  err->error = fsc::vee(rotmat_sp.transpose() * rotmat -
                        rotmat.transpose() * rotmat_sp) /
               2;

  const Eigen::Vector3d body_rate_sp = -2.0 / time_constant_ * err->error;

  ControlResult result;
  return {VehicleInput{-1.0, body_rate_sp}, ControllerErrc::kSuccess};
}

bool NonlinearGeometricController::setParams(const ParameterBase& params,
                                             LoggerBase& logger) {
  if (params.parameterFor() != "simple_attitude_controller") {
    logger.log(Severity::kError, "Mismatch in parameter and receiver");
    return false;
  }
  time_constant_ =
      static_cast<const NonlinearGeometricControllerParameters&>(params)
          .time_constant;
  return true;
}

std::shared_ptr<ParameterBase> NonlinearGeometricController::getParams(
    bool use_default) const {
  if (use_default) {
    return std::make_shared<NonlinearGeometricControllerParameters>();
  }

  NonlinearGeometricControllerParameters params;
  params.time_constant = time_constant_;
  return std::make_shared<NonlinearGeometricControllerParameters>(
      std::move(params));
}

bool NonlinearGeometricControllerParameters::valid(LoggerBase& logger) const {
  if (time_constant < 0.0) {
    logger.log(Severity::kError, "`time_constant` must be positive");
    return false;
  }
  return true;
}

std::string NonlinearGeometricControllerParameters::toString() const {
  std::ostringstream oss;
  oss << "Simple Attitude Controller parameters:\ntime_constant: "
      << time_constant;
  return oss.str();
}

std::string NonlinearGeometricControllerParameters::parameterFor() const {
  return "simple_attitude_controller";
}

bool NonlinearGeometricControllerParameters::load(
    const ParameterLoaderBase& loader, LoggerBase& logger) {
  std::ignore = loader.getParam("time_constant", time_constant);
  return true;
}

REGISTER_ATTITUDE_CONTROLLER(NonlinearGeometricController, "simple");

}  // namespace fsc
