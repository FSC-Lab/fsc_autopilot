#include "fsc_autopilot/attitude_control/px4_attitude_controller.hpp"

#include "fsc_autopilot/attitude_control/attitude_controller_factory.hpp"
#include "fsc_autopilot/attitude_control/control.hpp"
#include "fsc_autopilot/math/math_extras.hpp"

namespace fsc {

AttitudeControlResult PX4AttitudeController::run(
    const VehicleState& state, const AttitudeReference& refs, double dt,
    [[maybe_unused]] ContextBase* error) {
  using std::abs;
  using std::asin;
  using std::atan2;
  using std::sin;
  using std::sqrt;
  const auto& q = state.pose.orientation;
  Eigen::Quaterniond qd = refs.orientation;
  const auto yawspeed_setpoint = refs.yaw_rate;

  const Eigen::Quaterniond qe = q.inverse() * qd;

  Eigen::Quaterniond qe_red;
  Eigen::Vector3d eq;
  double yaw_e_angle;
  const double w_sq = pown<2>(qe.w()) + pown<2>(qe.z());
  if (IsClose(w_sq, 0.0)) {
    qe_red = {0.0, qe.x(), qe.y(), 0.0};
    yaw_e_angle = 0.0;

  } else {
    const auto w = sqrt(w_sq);
    const auto iw = 1.0 / w;
    const auto qe0_by_w = iw * qe.w();
    const auto qe3_by_w = iw * qe.z();

    yaw_e_angle =
        2.0 * (qe.w() < 0.0 ? atan2(-qe.z(), -qe.w()) : atan2(qe.z(), qe.w()));
    qe_red = {w, qe0_by_w * qe.x() - qe3_by_w * qe.y(),
              qe0_by_w * qe.y() + qe3_by_w * qe.x(), 0.0};
  }

  // calculate angular rates setpoint
  const Eigen::Vector3d angle_error =
      QuaternionToAngleAxis(qe_red) + yaw_e_angle * Eigen::Vector3d::UnitZ();
  Eigen::Vector3d ang_vel_target = kp_angle_.cwiseProduct(angle_error);

  // Feed forward the yaw setpoint rate.
  // yawspeed_setpoint is the feed forward commanded rotation around the world
  // z-axis, but we need to apply it in the body frame (because _rates_sp is
  // expressed in the body frame). Therefore we infer the world z-axis
  // (expressed in the body frame) by taking the last column of R.transposed (==
  // q.inversed) and multiply it by the yaw setpoint rate (yawspeed_setpoint).
  // This yields a vector representing the commanded rotatation around the world
  // z-axis expressed in the body frame such that it can be added to the rates
  // setpoint.
  Eigen::Vector3d rate_feedforward = Eigen::Vector3d::Zero();
  if (std::isfinite(yawspeed_setpoint) && !IsClose(yawspeed_setpoint, 0.0)) {
    rate_feedforward =
        q.inverse().toRotationMatrix().col(2) * yawspeed_setpoint;
    ang_vel_target += rate_feedforward;
  }

  // limit rates
  ang_vel_target = apm::BodyRateLimiting(ang_vel_target, ang_vel_max_);
  AttitudeControllerState* err;
  if ((error != nullptr) && error->name() == "attitude_controller_state") {
    err = static_cast<decltype(err)>(error);

    err->reference = qd;
    err->feedback = q;
    err->attitude_error = qe;
    err->error = angle_error;
    err->rate_feedforward = rate_feedforward;
    err->output = ang_vel_target;
  }

  return {VehicleInput{0.0, ang_vel_target}, ControllerErrc::kSuccess};
}

std::shared_ptr<ParameterBase> PX4AttitudeController::getParams(
    bool use_default) const {
  if (use_default) {
    return std::make_shared<PX4AttitudeControllerParams>();
  }
  PX4AttitudeControllerParams params;
  params.kp_angle = kp_angle_;
  params.ang_vel_max = ang_vel_max_;
  params.yaw_weight = yaw_weight_;
  return std::make_shared<PX4AttitudeControllerParams>(params);
}

bool PX4AttitudeController::setParams(const ParameterBase& params,
                                      LoggerBase& logger) {
  if (params.parameterFor() != "px4_attitude_controller") {
    logger.log(Severity::kError, "Mismatch in parameter and receiver");
    return false;
  }
  if (!params.valid(logger)) {
    return false;
  }
  const auto& p = static_cast<const PX4AttitudeControllerParams&>(params);
  kp_angle_ = p.kp_angle;
  ang_vel_max_ = p.ang_vel_max;
  yaw_weight_ = p.yaw_weight;

  return true;
}

bool PX4AttitudeControllerParams::valid(LoggerBase& logger) const {
  return (kp_angle.array() > 0.0).all() && yaw_weight > 0.0;
}

bool PX4AttitudeControllerParams::load(const ParameterLoaderBase& loader,
                                       LoggerBase& logger) {
  std::ignore = loader.getParam("roll_p", kp_angle.x());
  std::ignore = loader.getParam("pitch_p", kp_angle.y());
  std::ignore = loader.getParam("yaw_p", kp_angle.z());
  std::ignore = loader.getParam("yaw_weight", yaw_weight);
  return true;
}

std::string PX4AttitudeControllerParams::toString() const {
  std::ostringstream oss;
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};

  oss << "PX4 Attitude Controller parameters:"                    //
      << "\nkp_angle: " << kp_angle.transpose().format(f)         //
      << "\nang_vel_max: " << ang_vel_max.transpose().format(f);  //
  return oss.str();
}

REGISTER_ATTITUDE_CONTROLLER(PX4AttitudeController, "px4");

}  // namespace fsc
