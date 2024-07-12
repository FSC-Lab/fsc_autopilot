#ifndef TRACKING_CONTROL_UDE_VELOCITY_BASED_MULTIROTOR_UDE_HPP_
#define TRACKING_CONTROL_UDE_VELOCITY_BASED_MULTIROTOR_UDE_HPP_

#include "fsc_autopilot/ude/ude_base.hpp"
namespace fsc {

class VelocityBasedUDE : public UDEBase {
 public:
  using UDEBase::UDEBase;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   UDEState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(const VehicleState& state,
                                               UDEState* err) const override;

  [[nodiscard]] bool isVelocityBased() const override { return true; }
};

class BodyVelocityBasedUDE : public UDEBase {
 public:
  using UDEBase::UDEBase;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   UDEState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(const VehicleState& state,
                                               UDEState* err) const override;

  [[nodiscard]] bool isVelocityBased() const override { return true; }
};

}  // namespace fsc

#endif  // TRACKING_CONTROL_UDE_VELOCITY_BASED_MULTIROTOR_UDE_HPP_
