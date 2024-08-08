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

#ifndef FSC_AUTOPILOT_LQG_VELOCITY_BASED_LQG_HPP_
#define FSC_AUTOPILOT_LQG_VELOCITY_BASED_LQG_HPP_

#include <string>

#include "fsc_autopilot/lqg/lqg_base.hpp"

namespace fsc {

class VelocityBasedLQG : public LQGBase {
 public:
  using LQGBase::LQGBase;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   LQGState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(const VehicleState& state,
                                               LQGState* err) const override;

  [[nodiscard]] std::string type() const override { return "velocity_based"; }
};

class BodyVelocityBasedLQG : public LQGBase {
 public:
  using LQGBase::LQGBase;
  Eigen::Vector3d computeIntegrand(const VehicleState& state,
                                   const VehicleInput& input,
                                   LQGState* err) const override;

  [[nodiscard]] Eigen::Vector3d computeDamping(const VehicleState& state,
                                               LQGState* err) const override;

  [[nodiscard]] std::string type() const override {
    return "body_velocity_based";
  }
};

}  // namespace fsc

#endif  // FSC_AUTOPILOT_LQG_VELOCITY_BASED_LQG_HPP_
