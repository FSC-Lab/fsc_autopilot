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

#ifndef FSC_AUTOPILOT_CORE_VEHICLE_INPUT_HPP_
#define FSC_AUTOPILOT_CORE_VEHICLE_INPUT_HPP_

#include <utility>
#include <variant>

#include "Eigen/Dense"

namespace fsc {

struct ThrustRates {
  double thrust{0.0};
  Eigen::Vector3d body_rates{Eigen::Vector3d::Zero()};
};

struct ThrustAttitude {
  double thrust{0.0};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class VehicleInput {
 public:
  using CommandType = std::variant<ThrustRates, ThrustAttitude>;
  VehicleInput() = default;

  explicit VehicleInput(CommandType command) : command_(std::move(command)) {}

  explicit VehicleInput(const ThrustRates& thrust_rates)
      : command_(thrust_rates) {}

  explicit VehicleInput(const ThrustAttitude& thrust_rates)
      : command_(thrust_rates) {}

  template <typename Derived>
  explicit VehicleInput(const double thrust,
                        const Eigen::MatrixBase<Derived>& body_rates)
      : command_(ThrustRates{thrust, body_rates}) {}

  template <typename Derived>
  explicit VehicleInput(const double thrust,
                        const Eigen::QuaternionBase<Derived>& attitude)
      : command_(ThrustAttitude{thrust, attitude}) {}

  [[nodiscard]] const std::variant<ThrustRates, ThrustAttitude>& command()
      const {
    return command_;
  }

  std::variant<ThrustRates, ThrustAttitude>& command() { return command_; }

  ThrustRates& thrust_rates() { return std::get<ThrustRates>(command_); }

  [[nodiscard]] const ThrustRates& thrust_rates() const {
    return std::get<ThrustRates>(command_);
  }

  ThrustAttitude& body_rates() { return std::get<ThrustAttitude>(command_); }

  [[nodiscard]] const ThrustAttitude& thrust_attitude() const {
    return std::get<ThrustAttitude>(command_);
  }

 private:
  CommandType command_;
};

}  // namespace fsc

#endif  // FSC_AUTOPILOT_CORE_VEHICLE_INPUT_HPP_
