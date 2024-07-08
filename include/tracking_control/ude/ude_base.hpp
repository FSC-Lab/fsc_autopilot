#ifndef TRACKING_CONTROL_UDE_MULTIROTOR_UDE_HPP_
#define TRACKING_CONTROL_UDE_MULTIROTOR_UDE_HPP_

#include <memory>
#include <string>

#include "tracking_control/controller_base.hpp"
#include "tracking_control/definitions.hpp"
#include "tracking_control/logging.hpp"
#include "tracking_control/vehicle_input.hpp"

namespace fsc {
enum class UDEErrc;
}

namespace std {
template <>
struct is_error_code_enum<::fsc::UDEErrc> : true_type {};
}  // namespace std

namespace fsc {

enum class UDEErrc {
  kSuccess = 0,
  kInvalidState = 1,
  kInvalidInput = 2,
  kInvalidParameters = 3,
  kComputationError = 4,
};

namespace detail {
class UDEErrcCategory final : public std::error_category {
 public:
  [[nodiscard]] const char* name() const noexcept final { return "UDEError"; }

  [[nodiscard]] std::string message(int c) const final {
    switch (static_cast<fsc::UDEErrc>(c)) {
      case fsc::UDEErrc::kSuccess:
        return "UDE successful";
      case fsc::UDEErrc::kInvalidState:
        return "system state is invalid";
      case fsc::UDEErrc::kInvalidInput:
        return "reference is invalid";
      case fsc::UDEErrc::kInvalidParameters:
        return "UDE parameters are invalid";
      case fsc::UDEErrc::kComputationError:
        return "error in UDE computation";
      default:
        return "Invalid error code";
    }
  }
};
}  // namespace detail

extern inline const detail::UDEErrcCategory& GetUDEErrcCategory() {
  static detail::UDEErrcCategory c;
  return c;
}

inline std::error_code make_error_code(fsc::UDEErrc errc) {
  return {static_cast<int>(errc), GetUDEErrcCategory()};
}

struct UDEResult {
  Eigen::VectorXd ude_value;
  std::error_code ec;
};

enum class UDEType {
  kVelocityBased,
  kAccelBased,
  kBodyVelocityBased,
  kBodyAccelBased,
};

struct UDEParameters : public ParameterBase {
  std::string type_str;

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

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase* logger) override;

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string parameterFor() const override { return type_str; }
};

struct UDEState final : public ContextBase {
  std::string type_str;
  bool is_flying;
  bool is_active;
  Eigen::Vector3d damping_term{Eigen::Vector3d::Zero()};
  Eigen::Vector3d actuation_term{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dynamical_term{Eigen::Vector3d::Zero()};
  Eigen::Vector3d integral{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbance_estimate{Eigen::Vector3d::Zero()};

  [[nodiscard]] std::string name() const override { return "ude.error"; }
};

class UDEBase {
 public:
  using Parameters = UDEParameters;
  using ParametersSharedPtr = std::shared_ptr<Parameters>;
  using ParametersConstSharedPtr = std::shared_ptr<const Parameters>;

  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  explicit UDEBase(ParametersSharedPtr params);

  UDEErrc update(const VehicleState& state, const VehicleInput& input,
                 ContextBase* error);

  [[nodiscard]] ParametersConstSharedPtr params() const { return params_; }
  ParametersSharedPtr& params() { return params_; }

  [[nodiscard]] virtual bool getEstimate(
      Eigen::Ref<Eigen::VectorXd> estimate) const;

  [[nodiscard]] virtual bool isVelocityBased() const = 0;

 protected:
  virtual Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                           const VehicleInput& input,
                                           UDEState* err) const = 0;

  virtual Eigen::Vector3d computeDamping(const VehicleState& state,
                                         UDEState* err) const = 0;

  ParametersSharedPtr params_{std::make_shared<Parameters>()};
  Eigen::Vector3d ude_integral_;
  Eigen::Vector3d ude_value_;
};

}  // namespace fsc
#endif  // TRACKING_CONTROL_UDE_MULTIROTOR_UDE_HPP_
