#include "tracking_control/nonlinear_geometric_controller.hpp"

#include "tracking_control/definitions.hpp"
#include "tracking_control/math.hpp"
#include "tracking_control/vehicle_input.hpp"

namespace fsc {

NonlinearGeometricController::NonlinearGeometricController(Parameters params)
    : params_(params) {}

ControlResult NonlinearGeometricController::run(
    const VehicleState& state, const Reference& refs,
    [[maybe_unused]] ControlErrorBase* error) {
  const auto rotmat = state.pose.orientation.toRotationMatrix();
  const auto rotmat_sp = refs.state.pose.orientation.toRotationMatrix();

  NonlinearGeometricControllerError* err;
  if ((error != nullptr) &&
      error->name() == "nonlinear_geometric_controller.error") {
    err = static_cast<decltype(err)>(error);
  }
  // e_r = 1 / 2 * (Rd.T * R - R.T * Rd)
  err->attitude_error = fsc::vee(rotmat_sp.transpose() * rotmat -
                                 rotmat.transpose() * rotmat_sp) /
                        2;

  const Eigen::Vector3d body_rate_sp =
      -2.0 / params_.time_constant * err->attitude_error;

  ControlResult result;
  result.ec = ControllerErrc::kSuccess;
  result.setpoint.input = VehicleInput{-1.0, body_rate_sp};
  return result;
}
}  // namespace fsc
