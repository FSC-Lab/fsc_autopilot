#include <memory>
#include <string>

#include "Eigen/Dense"
#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"

namespace fsc {

struct PX4AttitudeControllerParams final : public ParameterBase {
  using ParameterBase::load;
  [[nodiscard]] bool valid(LoggerBase& logger) const override;

  [[nodiscard]] std::string parameterFor() const override {
    return "px4_attitude_controller";
  }

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase& logger) override;

  [[nodiscard]] std::string toString() const override;

  static constexpr double kDefaultYawWeight{0.4};

  double yaw_weight{kDefaultYawWeight};

  Eigen::Vector3d kp_angle;
  Eigen::Vector3d ang_vel_max{Eigen::Vector3d::Zero()};
};

class PX4AttitudeController : public AttitudeControllerBase {
 public:
  using Parameters = PX4AttitudeControllerParams;
  using ParametersSharedPtr = std::shared_ptr<PX4AttitudeControllerParams>;

  PX4AttitudeController() = default;

  AttitudeControlResult run(const VehicleState& state,
                            const AttitudeReference& refs, double dt,
                            [[maybe_unused]] ContextBase* error) override;

  [[nodiscard]] std::string name() const override {
    return "px4_attitude_controller";
  }

  [[nodiscard]] std::shared_ptr<ParameterBase> getParams(
      bool use_default) const override;

  [[nodiscard]] bool setParams(const ParameterBase& params,
                               LoggerBase& logger) override;

 private:
  void decomposeAttitudeError(const Eigen::Quaterniond& qe,
                              Eigen::Quaterniond& qe_red,
                              double& yaw_e_angle) const;

  Eigen::Vector3d kp_angle_;
  Eigen::Vector3d ang_vel_max_;
  double yaw_weight_;
};
}  // namespace fsc
