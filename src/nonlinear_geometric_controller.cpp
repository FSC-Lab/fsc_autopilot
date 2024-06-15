#include "tracking_control/nonlinear_geometric_controller.hpp"

#include "tracking_control/internal/internal.hpp"

namespace fsc {

NonlinearGeometricController::NonlinearGeometricController(Parameters params)
    : params_(params) {}

ControlResult NonlinearGeometricController::run(const VehicleState& state,
                                                const Setpoint& refs,
                                                double dt) {
  const auto rotmat = state.pose.orientation.toRotationMatrix();
  const auto rotmat_sp = refs.state.pose.orientation.toRotationMatrix();

  auto error = std::make_shared<Error>();
  // e_r = 1 / 2 * (Rd.T * R - R.T * Rd)
  error->attitude_error = fsc::vee(rotmat_sp.transpose() * rotmat -
                                   rotmat.transpose() * rotmat_sp) /
                          2;

  const Eigen::Vector3d body_rate_sp =
      -2.0 / params_.time_constant * error->attitude_error;

  ControlResult result;
  result.success = true;
  result.setpoint.input.command = body_rate_sp;
  result.error = std::move(error);
  return result;
}
}  // namespace fsc
