#ifndef TRACKING_CONTROL_UDE_BASE_HPP_
#define TRACKING_CONTROL_UDE_BASE_HPP_

#include <string>
#include <system_error>
#include <type_traits>

#include "tracking_control/definitions.hpp"
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

class UDEBase {
 public:
  virtual ~UDEBase() = default;

  virtual bool init([[maybe_unused]] const VehicleState& state) { return true; }

  virtual UDEErrc update(const VehicleState& state, const VehicleInput& input,
                         ContextBase* error) = 0;

  [[nodiscard]] virtual bool getEstimate(
      Eigen::Ref<Eigen::VectorXd> estimate) const = 0;

 private:
};

}  // namespace fsc
#endif  // TRACKING_CONTROL_UDE_BASE_HPP_
