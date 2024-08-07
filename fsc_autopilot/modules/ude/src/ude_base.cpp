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

#include "fsc_autopilot/ude/ude_base.hpp"

#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"

namespace fsc {

UDEErrc UDEBase::update(const VehicleState& state, const VehicleInput& input,
                        double dt, ContextBase* error) {
  UDEState* err;
  if (error->name() == "ude.error") {
    err = static_cast<decltype(err)>(error);
  }
  // safety logic:
  // if int_flag && alti_threshold == true: enable integration
  // if int_flag == false : reset integration
  // if alti_threshold == false && int_flag == true: hold integration

  const bool int_flag = ude_active_;
  const bool is_flying = (state.pose.position.z() > ude_height_threshold_);
  if (!int_flag) {
    ude_integral_.setZero();
  } else if (int_flag && is_flying) {
    // disturbance estimator
    ude_integral_ += ude_gain_ * computeIntegrand(state, input, err) * dt;
  }

  ude_value_ = ude_integral_ + computeDamping(state, err);

  ude_value_ = ude_value_.cwiseMax(ude_lb_).cwiseMin(ude_ub_);

  if (err) {
    err->type_str = type();
    err->is_active = ude_active_;
    err->is_flying = is_flying;
    err->integral = ude_integral_;
    err->disturbance_estimate = ude_value_;
  }
  return UDEErrc::kSuccess;
}

bool UDEBase::setParams(const UDEParameters& params, LoggerBase& logger) {
  if (!params.valid(logger)) {
    return false;
  }
  vehicle_mass_ = params.vehicle_mass;
  ude_height_threshold_ = params.ude_height_threshold;

  ude_gain_ = params.ude_gain;
  ude_lb_ = params.ude_lb;
  ude_ub_ = params.ude_ub;
  return true;
}

bool UDEBase::getEstimate(Eigen::Ref<Eigen::VectorXd> estimate) const {
  estimate = ude_value_;
  return true;
}

std::string UDEParameters::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};
  std::ostringstream oss;

  oss << "UDE parameters:"                               //
      << "\nvehicle_mass: " << vehicle_mass              //
      << "\nheight_threshold: " << ude_height_threshold  //
      << "\nde_gain: " << ude_gain                       //
      << "\nde_lb: " << ude_lb.transpose().format(f)     //
      << "\nde_ub: " << ude_ub.transpose().format(f);    //
  return oss.str();
}

bool UDEParameters::load(const ParameterLoaderBase& loader,
                         LoggerBase& logger) {
  ude_lb << loader.param("lbx", ude_lb.x()),  //
      loader.param("lby", ude_lb.y()),        //
      loader.param("lbz", ude_lb.z());

  ude_ub << loader.param("ubx", ude_ub.x()),  //
      loader.param("uby", ude_ub.y()),        //
      loader.param("ubz", ude_ub.z());

  std::ignore = loader.getParam("height_threshold", ude_height_threshold);
  std::ignore = loader.getParam("gain", ude_gain);
  if (!loader.getParam("vehicle_mass", vehicle_mass)) {
    logger.log(Severity::kError, "Failed to load parameter `vehicle_mass`");

    return false;
  }

  if (!valid(logger)) {
    logger.log(Severity::kError) << "Invalid parameters!\n" << toString();
    return false;
  }
  return true;
}

}  // namespace fsc
