#include "tracking_control/ude/ude_base.hpp"

#include "tracking_control/definitions.hpp"
#include "tracking_control/vehicle_input.hpp"

namespace fsc {
UDEBase::UDEBase(ParametersSharedPtr params) : params_(std::move(params)) {}

UDEErrc UDEBase::update(const VehicleState& state, const VehicleInput& input,
                        ContextBase* error) {
  if (params_ == nullptr || !params_->valid()) {
    return UDEErrc::kInvalidParameters;
  }
  UDEState* err;
  if (error->name() == "ude.error") {
    err = static_cast<decltype(err)>(error);
  }
  // safety logic:
  // if int_flag && alti_threshold == true: enable integration
  // if int_flag == false : reset integration
  // if alti_threshold == false && int_flag == true: hold integration

  const bool int_flag = params_->ude_active;
  const bool is_flying =
      (state.pose.position.z() > params_->ude_height_threshold);
  if (!int_flag) {
    ude_integral_.setZero();
  } else if (int_flag && is_flying) {
    // disturbance estimator
    ude_integral_ +=
        params_->ude_gain * computeIntegrand(state, input, err) * params_->dt;
  }

  ude_value_ = ude_integral_ + computeDamping(state, err);

  ude_value_ = ude_value_.cwiseMax(params_->ude_lb).cwiseMin(params_->ude_ub);

  if (err) {
    err->type_str = params_->type_str;
    err->is_active = params_->ude_active;
    err->is_flying = is_flying;
    err->integral = ude_integral_;
    err->disturbance_estimate = ude_value_;
  }
  return UDEErrc::kSuccess;
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
      << "dt: " << dt                                    //
      << "\nheight_threshold: " << ude_height_threshold  //
      << "\nde_gain: " << ude_gain                       //
      << "\nde_lb: " << ude_lb.transpose().format(f)     //
      << "\nde_ub: " << ude_ub.transpose().format(f);    //
  return oss.str();
}

}  // namespace fsc
