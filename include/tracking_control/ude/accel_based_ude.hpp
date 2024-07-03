#ifndef TRACKING_CONTROL_UDE_ACCEL_BASED_MULTIROTOR_UDE_HPP_
#define TRACKING_CONTROL_UDE_ACCEL_BASED_MULTIROTOR_UDE_HPP_

#include "tracking_control/ude/ude_base.hpp"

namespace fsc {
class AccelBasedUDE : public UDEBase {
 public:
  using UDEBase::UDEBase;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   UDEState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(
      [[maybe_unused]] const VehicleState& state, UDEState* err) const override;

  [[nodiscard]] bool isVelocityBased() const override { return false; }
};

class BodyAccelBasedUDE : public UDEBase {
 public:
  using UDEBase::UDEBase;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   UDEState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(
      [[maybe_unused]] const VehicleState& state, UDEState* err) const override;

  [[nodiscard]] bool isVelocityBased() const override { return false; }
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_UDE_ACCEL_BASED_MULTIROTOR_UDE_HPP_