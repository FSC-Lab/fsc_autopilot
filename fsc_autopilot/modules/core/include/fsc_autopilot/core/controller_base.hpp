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

#ifndef FSC_AUTOPILOT_CORE_CONTROLLER_BASE_HPP_
#define FSC_AUTOPILOT_CORE_CONTROLLER_BASE_HPP_

#include <string>
#include <system_error>
#include <type_traits>

#include "Eigen/Dense"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"

namespace fsc {
enum class ControllerErrc;
}

namespace std {

template <>
struct is_error_code_enum<::fsc::ControllerErrc> : true_type {};

}  // namespace std

namespace fsc {

enum class ControllerErrc {
  kSuccess = 0,
  kInvalidState = 1,
  kInvalidReference = 2,
  kInvalidParameters = 3,
  kComputationError = 4,
  kSubcomponentMissing = 5,
  kSubcomponentError = 6,
};

namespace detail {
class ControllerErrcCategory final : public std::error_category {
 public:
  [[nodiscard]] const char* name() const noexcept final {
    return "ControllerError";
  }

  [[nodiscard]] std::string message(int c) const final {
    switch (static_cast<fsc::ControllerErrc>(c)) {
      case fsc::ControllerErrc::kSuccess:
        return "controller successful";
      case fsc::ControllerErrc::kInvalidState:
        return "system state is invalid";
      case fsc::ControllerErrc::kInvalidReference:
        return "reference is invalid";
      case fsc::ControllerErrc::kInvalidParameters:
        return "controller parameters are invalid";
      case fsc::ControllerErrc::kComputationError:
        return "error in controller computation";
      case fsc::ControllerErrc::kSubcomponentMissing:
        return "controller subcomponent is missing";
      case fsc::ControllerErrc::kSubcomponentError:
        return "error in a controller subcomponent";

      default:
        return "Invalid error code";
    }
  }
};
}  // namespace detail

extern inline const detail::ControllerErrcCategory&
GetControllerErrcCategory() {
  static detail::ControllerErrcCategory c;
  return c;
}

inline std::error_code make_error_code(fsc::ControllerErrc errc) {
  return {static_cast<int>(errc), GetControllerErrcCategory()};
}

struct ControlResult {
  Setpoint setpoint;
  std::error_code ec;

  explicit operator bool() const noexcept {
    return ec == ControllerErrc::kSuccess;
  }
};

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual ControlResult run(const VehicleState& state, const Reference& refs,
                            ContextBase* error) = 0;

  ControlResult run(const VehicleState& state, const Reference& refs) {
    return run(state, refs, nullptr);
  }

  [[nodiscard]] virtual Setpoint getFallBackSetpoint() const {
    return {VehicleState{}, VehicleInput{0.0, Eigen::Quaterniond::Identity()}};
  }

  [[nodiscard]] virtual std::string name() const = 0;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_CORE_CONTROLLER_BASE_HPP_
