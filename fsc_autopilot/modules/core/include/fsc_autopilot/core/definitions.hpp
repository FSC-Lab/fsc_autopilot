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

#ifndef FSC_AUTOPILOT_CORE_DEFINITIONS_HPP_
#define FSC_AUTOPILOT_CORE_DEFINITIONS_HPP_
#include <string>

#include "Eigen/Core"      // IWYU pragma: keep
#include "Eigen/Geometry"  // IWYU pragma: keep
#include "fsc_autopilot/core/vehicle_input.hpp"

namespace fsc {
struct Pose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct Twist {
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct Accel {
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct VehicleState {
  double stamp;
  Pose pose;
  Twist twist;
  Accel accel;
};

enum class ReferenceKind {
  kPosition = 1 << 0,
  kAttitude = 1 << 1,
  kVelocity = 1 << 2,
  kBodyRate = 1 << 3
};

struct Reference {
  VehicleState state;
  double yaw{0.0};
  double yaw_rate{0.0};
  ReferenceKind kind;
};

struct Setpoint {
  VehicleState state;
  VehicleInput input;
};

struct ContextBase {
  virtual ~ContextBase() = default;

  [[nodiscard]] virtual std::string message() const { return ""; }

  [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace fsc

#endif  // FSC_AUTOPILOT_CORE_DEFINITIONS_HPP_
