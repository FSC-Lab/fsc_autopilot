// Copyright © 2024 FSC Lab
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef FSC_AUTOPILOT_UDE_UDE_BASE_HPP_
#define FSC_AUTOPILOT_UDE_UDE_BASE_HPP_

#include <memory>
#include <string>

#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"

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
  using ParameterBase::load;

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
    return (ude_lb.array() < ude_ub.array()).all() && vehicle_mass > 0.0;
  }

  [[nodiscard]] bool load(const ParameterLoaderBase& loader,
                          LoggerBase* logger) override;

  [[nodiscard]] std::string toString() const override;

  [[nodiscard]] std::string parameterFor() const override { return "ude"; }
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
  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  UDEBase() = default;

  UDEErrc update(const VehicleState& state, const VehicleInput& input,
                 double dt, ContextBase* error);

  bool setParams(const UDEParameters& params);

  [[nodiscard]] virtual bool getEstimate(
      Eigen::Ref<Eigen::VectorXd> estimate) const;

  [[nodiscard]] virtual std::string type() const = 0;

 protected:
  virtual Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                           const VehicleInput& input,
                                           UDEState* err) const = 0;

  virtual Eigen::Vector3d computeDamping(const VehicleState& state,
                                         UDEState* err) const = 0;

  bool ude_active_;

  double vehicle_mass_;
  double ude_height_threshold_;
  double ude_gain_;
  Eigen::Vector3d ude_lb_;
  Eigen::Vector3d ude_ub_;

  Eigen::Vector3d ude_integral_;
  Eigen::Vector3d ude_value_;
};

}  // namespace fsc
#endif  // FSC_AUTOPILOT_UDE_UDE_BASE_HPP_
