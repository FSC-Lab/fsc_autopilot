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

#include "fsc_autopilot/lqg/lqg_base.hpp"

#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"

namespace fsc {

LQGErrc LQGBase::update(const VehicleState& state, const VehicleInput& input,
                        double dt, ContextBase* error) {
  LQGState* err;
  if (error->name() == "lqg.error") {
    err = static_cast<decltype(err)>(error);
  }
  // safety logic:
  // if int_flag && alti_threshold == true: enable integration
  // if int_flag == false : reset integration
  // if alti_threshold == false && int_flag == true: hold integration

  const bool int_flag = lqg_active_;
  const bool is_flying = (state.pose.position.z() > lqg_height_threshold_);
  if (!int_flag) {
    lqg_integral_.setZero();
  } else if (int_flag && is_flying) {
    // disturbance estimator
    lqg_integral_ += lqg_gain_ * computeIntegrand(state, input, err) * dt;
  }

  lqg_value_ = lqg_integral_ + computeDamping(state, err);

  lqg_value_ = lqg_value_.cwiseMax(lqg_lb_).cwiseMin(lqg_ub_);

  if (err) {
    err->type_str = type();
    err->is_active = lqg_active_;
    err->is_flying = is_flying;
    err->integral = lqg_integral_;
    err->disturbance_estimate = lqg_value_;
  }
  return LQGErrc::kSuccess;
}

bool LQGBase::setParams(const LQGParameters& params) {
  if (!params.valid()) {
    return false;
  }
  vehicle_mass_ = params.vehicle_mass;
  lqg_height_threshold_ = params.lqg_height_threshold;

  lqg_gain_ = params.lqg_gain;
  lqg_lb_ = params.lqg_lb;
  lqg_ub_ = params.lqg_ub;
  return true;
}

bool LQGBase::getEstimate(Eigen::Ref<Eigen::VectorXd> estimate) const {
  estimate = lqg_value_;
  return true;
}

std::string LQGParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "LQG parameters:"                               //
      << "\nvehicle_mass: " << vehicle_mass              //
      << "\nheight_threshold: " << lqg_height_threshold  //
      << "\nde_gain: " << lqg_gain                       //
      << "\nde_lb: " << lqg_lb.transpose().format(f)     //
      << "\nde_ub: " << lqg_ub.transpose().format(f);    //
  return oss.str();
}

bool LQGParameters::load(const ParameterLoaderBase& loader,
                         LoggerBase* logger) {
  lqg_lb << loader.param("lbx", lqg_lb.x()),  //
      loader.param("lby", lqg_lb.y()),        //
      loader.param("lbz", lqg_lb.z());

  lqg_ub << loader.param("lbx", lqg_ub.x()),  //
      loader.param("lby", lqg_ub.y()),        //
      loader.param("lbz", lqg_ub.z());

  std::ignore = loader.getParam("height_threshold", lqg_height_threshold);
  std::ignore = loader.getParam("gain", lqg_gain);
  if (!loader.getParam("vehicle_mass", vehicle_mass)) {
    if (logger) {
      logger->log(Severity::kError, "Failed to load parameter `vehicle_mass`");
    }

    return false;
  }
  return true;
}

}  // namespace fsc
