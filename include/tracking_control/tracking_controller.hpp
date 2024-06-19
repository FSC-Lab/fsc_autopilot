#ifndef TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_

#include <iostream>
#include <string>

#include "Eigen/Dense"
#include "fmt/format.h"
#include "fmt/ostream.h"
#include "tracking_control/controller_base.hpp"
#include "utils/utils.hpp"

namespace fsc {

struct TrackingControllerError final : public ControlErrorBase {
  [[nodiscard]] std::string name() const final {
    return "tracking_controller.error";
  }
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ude_output{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_sp{Eigen::Vector3d::Zero()};
  bool ude_effective{false};
};

class TrackingControllerContext : public Context {
 public:
  [[nodiscard]] bool get(std::string_view name, bool& value) const override {
    if (name == "interrupt_ude") {
      value = interrupt_ude_;
      return true;
    }
    return false;
  }

  [[nodiscard]] bool set(std::string_view name, bool value) override {
    if (name == "interrupt_ude") {
      interrupt_ude_ = value;
      return true;
    }
    return false;
  }

  [[nodiscard]] bool get(std::string_view name, double& value) const override {
    if (name == "dt") {
      value = dt_;
      return true;
    }
    return false;
  }

  [[nodiscard]] bool set(std::string_view name, double value) override {
    if (name == "dt") {
      if (value < 0.0) {
        return false;
      }
      dt_ = value;
      return true;
    }
    return false;
  }

 private:
  bool interrupt_ude_;
  double dt_{-1.0};
};

struct TrackingControllerParameters {
  static constexpr double kDefaultKpXY{1.0};
  static constexpr double kDefaultKpZ{10.0};
  Eigen::Vector3d k_pos{kDefaultKpXY, kDefaultKpXY, kDefaultKpZ};

  static constexpr double kDefaultKvXY{1.5};
  static constexpr double kDefaultKvZ{3.3};
  Eigen::Vector3d k_vel{kDefaultKvXY, kDefaultKvXY, kDefaultKvZ};
  double min_z_accel{0};

  static constexpr double kDefaultMaxZAccel{20};
  double max_z_accel{kDefaultMaxZAccel};

  static constexpr double kDefaultMaxTiltAngle{45};
  double max_tilt_angle{kDefaultMaxTiltAngle};

  static constexpr double kDefaultDEGain{1.0};
  double de_gain{kDefaultDEGain};

  static constexpr double kDefaultDEHeightThreshold{0.1};
  double de_height_threshold{kDefaultDEHeightThreshold};

  static constexpr double kVehicleMassSentinel{-1.0};
  double vehicle_mass{kVehicleMassSentinel};

  uint32_t num_of_rotors{4};

  static constexpr double kDefaultDEBounds{5};
  Eigen::Vector3d de_lb{Eigen::Vector3d::Constant(-kDefaultDEBounds)};
  Eigen::Vector3d de_ub{Eigen::Vector3d::Constant(kDefaultDEBounds)};
};

class TrackingController final : public ControllerBase {
 public:
  inline static const Eigen::Vector3d kGravity{Eigen::Vector3d::UnitZ() * 9.81};

  TrackingController() = default;
  explicit TrackingController(TrackingControllerParameters params);

  ControlResult run(const VehicleState& state, const Reference& refs) override;

  [[nodiscard]] const TrackingControllerParameters& params() const {
    return params_;
  }
  TrackingControllerParameters& params() { return params_; }

  // Expose this member for RotorDragModel mixin

  // Expose these parameters for AccelerationSetpointShaping mixin
  [[nodiscard]] double min_z_accel() const { return params_.min_z_accel; }
  [[nodiscard]] double max_z_accel() const { return params_.max_z_accel; }
  [[nodiscard]] double max_tilt_angle() const { return params_.max_tilt_angle; }

  [[nodiscard]] std::string name() const final { return "tracking_controller"; }

 private:
  Eigen::Vector3d disturbance_estimate_{Eigen::Vector3d::Zero()};

  double thrust_sp_{0.0};
  TrackingControllerParameters params_;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_TRACKING_CONTROLLER_HPP_
