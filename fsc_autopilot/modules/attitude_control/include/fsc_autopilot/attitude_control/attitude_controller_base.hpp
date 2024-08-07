#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_BASE_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_BASE_HPP_

#include <string>

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

class AttitudeControllerState : public ContextBase {
 public:
  [[nodiscard]] std::string name() const override {
    return "attitude_controller_state";
  }

  Eigen::Quaterniond reference{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond feedback{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond attitude_error{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rate_feedforward{Eigen::Vector3d::Zero()};
  Eigen::Vector3d output{Eigen::Vector3d::Zero()};
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
