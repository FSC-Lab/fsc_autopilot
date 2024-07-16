#include <memory>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/core/controller_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"

namespace fsc {

struct PX4AttitudeControllerParams final : public ParameterBase {
  using ParameterBase::load;

  Eigen::Vector3d kp_angle;
  Eigen::Vector3d ang_vel_max{Eigen::Vector3d::Zero()};
  double yaw_weight{0.4};
  double dt;

  [[nodiscard]] bool valid() const override {
    return (kp_angle.array() > 0.0).all() && yaw_weight > 0.0;
  }

  [[nodiscard]] std::string parameterFor() const override {
    return "px4_attitude_controller";
  }

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase* logger) override;

  [[nodiscard]] std::string toString() const override;
};

class PX4AttitudeController : public ControllerBase {
 public:
  using Parameters = PX4AttitudeControllerParams;
  using ParametersSharedPtr = std::shared_ptr<PX4AttitudeControllerParams>;

  PX4AttitudeController() = default;

  ControlResult run(const VehicleState& state, const Reference& refs,
                    [[maybe_unused]] ContextBase* error) override;

  std::shared_ptr<PX4AttitudeControllerParams>& params() { return params_; }

  [[nodiscard]] std::string name() const override {
    return "px4_attitude_controller";
  }

 private:
  std::shared_ptr<PX4AttitudeControllerParams> params_;
  // Eigen::Vector3d kp_angle_;
  // Eigen::Vector3d ang_vel_max_;
  // double yaw_w_;
};
}  // namespace fsc
