#ifndef TRACKING_CONTROL_UDE_ACCEL_BASED_MULTIROTOR_UDE_HPP_
#define TRACKING_CONTROL_UDE_ACCEL_BASED_MULTIROTOR_UDE_HPP_

#include "tracking_control/ude/multirotor_ude.hpp"

namespace fsc {
class AccelBasedMultirotorUDE : public MultirotorUDE {
 public:
  using MultirotorUDE::MultirotorUDE;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   MultirotorUDEState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(
      [[maybe_unused]] const VehicleState& state,
      MultirotorUDEState* err) const override;

  [[nodiscard]] bool isVelocityBased() const override { return false; }
};

class BodyAccelBasedMultirotorUDE : public MultirotorUDE {
 public:
  using MultirotorUDE::MultirotorUDE;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   MultirotorUDEState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(
      [[maybe_unused]] const VehicleState& state,
      MultirotorUDEState* err) const override;

  [[nodiscard]] bool isVelocityBased() const override { return false; }
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_UDE_ACCEL_BASED_MULTIROTOR_UDE_HPP_
