#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_BASE_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_BASE_HPP_

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
namespace fsc {

struct AttitudeReference {
  Eigen::Quaterniond orientation;
  double yaw;
  double yaw_rate;
};

struct AttitudeControlResult {
  VehicleInput input;
  std::error_code ec;

  explicit operator bool() const noexcept {
    return ec == ControllerErrc::kSuccess;
  }
};

class AttitudeControllerBase : public ControllerBase {
 public:
  ControlResult run(const VehicleState& state, const Reference& refs, double dt,
                    ContextBase* error) override {
    auto res = run(state, {refs.state.pose.orientation}, dt, error);
    return {Setpoint{{}, res.input}, res.ec};
  }

  virtual AttitudeControlResult run(const VehicleState& state,
                                    const AttitudeReference& refs, double dt,
                                    ContextBase* error) = 0;

 private:
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_BASE_HPP_
