#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_

#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/core/controller_base.hpp"

namespace fsc {

struct NonlinearGeometricControllerError final : public ContextBase {
  Eigen::Vector3d attitude_error;

  [[nodiscard]] std::string name() const final {
    return "nonlinear_geometric_controller.error";
  }
};

struct NonlinearGeometricControllerParameters {
  double time_constant;
};

struct NonlinearGeometricController : public ControllerBase {
  using Parameters = NonlinearGeometricControllerParameters;
  using Error = NonlinearGeometricControllerError;

  NonlinearGeometricController() = default;

  explicit NonlinearGeometricController(Parameters params);

  ControlResult run(const VehicleState& state, const Reference& refs,
                    ContextBase* error) override;

  [[nodiscard]] const Parameters& params() const { return params_; }

  Parameters& params() { return params_; }

  [[nodiscard]] std::string name() const final {
    return "nonlinear_geometric_controller";
  }

 private:
  Parameters params_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
