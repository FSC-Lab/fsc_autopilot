#ifndef FSC_AUTOPILOT_POSITION_CONTROL_POSITION_CONTROLLER_BASE_HPP_
#define FSC_AUTOPILOT_POSITION_CONTROL_POSITION_CONTROLLER_BASE_HPP_

#include <string>

#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"

namespace fsc {

struct PositionControlReference {
  struct FeedForward {
    enum class Type { kNone, kAcceleration, kThrust } type;
    Eigen::Vector3d value{Eigen::Vector3d::Zero()};
  };
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  FeedForward feedforward{};
  double yaw;
};

struct PositionControlResult {
  ThrustAttitude input;
  std::error_code ec;

  explicit operator bool() const noexcept {
    return ec == ControllerErrc::kSuccess;
  }
};

class PositionControllerState : public ContextBase {
 public:
  [[nodiscard]] std::string name() const override {
    return "position_controller_state";
  }
};

class PositionControllerBase : public ControllerBase {
 public:
  ControlResult run(const VehicleState& state, const Reference& refs, double dt,
                    ContextBase* error) override {
    auto res = run(state,
                   PositionControlReference{refs.state.pose.position,
                                            refs.state.twist.linear},
                   dt, error);
    return {Setpoint{{}, VehicleInput{res.input}}, res.ec};
  }

  virtual PositionControlResult run(const VehicleState& state,
                                    const PositionControlReference& refs,
                                    double dt, ContextBase* error) = 0;
};

}  // namespace fsc

#endif  // FSC_AUTOPILOT_POSITION_CONTROL_POSITION_CONTROLLER_BASE_HPP_
