#ifndef TRACKING_CONTROL_MULTIROTOR_UDE_HPP_
#define TRACKING_CONTROL_MULTIROTOR_UDE_HPP_

#include <memory>
#include <string>

#include "tracking_control/controller_base.hpp"
#include "tracking_control/definitions.hpp"
#include "tracking_control/ude/ude_base.hpp"
#include "tracking_control/vehicle_input.hpp"

namespace fsc {

enum class MultirotorUDEType {
  kVelocityBased,
  kAccelBased,
  kBodyVelocityBased,
  kBodyAccelBased,
};

struct MultirotorUDEParams final : public ParameterBase {
  MultirotorUDEType type{MultirotorUDEType::kVelocityBased};

  double dt;
  static constexpr double kDefaultDEGain{1.0};
  double ude_gain{kDefaultDEGain};

  static constexpr double kDefaultDEHeightThreshold{0.1};
  double ude_height_threshold{kDefaultDEHeightThreshold};

  static constexpr double kVehicleMassSentinel{-1.0};
  double vehicle_mass{kVehicleMassSentinel};

  bool ude_active{true};

  static constexpr double kDefaultUDEBounds{5};
  Eigen::Vector3d ude_lb{Eigen::Vector3d::Constant(-kDefaultUDEBounds)};
  Eigen::Vector3d ude_ub{Eigen::Vector3d::Constant(kDefaultUDEBounds)};

  [[nodiscard]] bool valid() const override {
    return (ude_lb.array() < ude_ub.array()).all() && vehicle_mass > 0.0 &&
           dt > 0.0;
  }

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string name() const override {
    return "multirotor_ude.parameters";
  }
};

struct MultirotorUDEState final : public ContextBase {
  MultirotorUDEType type;
  bool is_flying;
  bool is_active;
  Eigen::Vector3d damping_term{Eigen::Vector3d::Zero()};
  Eigen::Vector3d actuation_term{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dynamical_term{Eigen::Vector3d::Zero()};
  Eigen::Vector3d integral{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbance_estimate{Eigen::Vector3d::Zero()};

  [[nodiscard]] std::string name() const override { return "ude.error"; }
};

class MultirotorUDE : public UDEBase {
 public:
  using Parameters = MultirotorUDEParams;
  using ParametersSharedPtr = std::shared_ptr<Parameters>;
  using ParametersConstSharedPtr = std::shared_ptr<const Parameters>;

  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  explicit MultirotorUDE(ParametersSharedPtr params);

  UDEErrc update(const VehicleState& state, const VehicleInput& input,
                 ContextBase* error) override;

  [[nodiscard]] ParametersConstSharedPtr params() const { return params_; }
  ParametersSharedPtr& params() { return params_; }

  [[nodiscard]] bool getEstimate(
      Eigen::Ref<Eigen::VectorXd> estimate) const override;

  [[nodiscard]] virtual bool isVelocityBased() const = 0;

 protected:
  virtual Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                           const VehicleInput& input,
                                           MultirotorUDEState* err) const = 0;

  virtual Eigen::Vector3d computeDamping(const VehicleState& state,
                                         MultirotorUDEState* err) const = 0;

  ParametersSharedPtr params_;
  Eigen::Vector3d ude_integral_;
  Eigen::Vector3d ude_value_;
};

}  // namespace fsc
#endif  // TRACKING_CONTROL_MULTIROTOR_UDE_HPP_
