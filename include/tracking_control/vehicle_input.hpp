#ifndef TRACKING_CONTROL_VEHICLE_INPUT_HPP_
#define TRACKING_CONTROL_VEHICLE_INPUT_HPP_

#include <variant>

#include "Eigen/Dense"

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
  explicit VehicleInput(const ThrustRates& thrust_rates)
      : command_(thrust_rates) {}

  VehicleInput() = default;

  template <typename Derived>
  explicit VehicleInput(const double thrust,
                        const Eigen::MatrixBase<Derived>& body_rates)
      : command_(ThrustRates{thrust, body_rates}) {}

  explicit VehicleInput(const ThrustAttitude& thrust_attitude)
      : command_(thrust_attitude) {}

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
  std::variant<ThrustRates, ThrustAttitude> command_;
};

#endif  // TRACKING_CONTROL_VEHICLE_INPUT_HPP_
