
#include "tracking_control/attitude_control/apm_attitude_controller.hpp"

#include <utility>

#include "Eigen/Dense"
#include "tracking_control/control.hpp"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/definitions.hpp"
#include "tracking_control/logging.hpp"
#include "tracking_control/vehicle_input.hpp"

namespace fsc {
APMAttitudeController::APMAttitudeController(ParametersSharedPtr params)
    : params_(std::move(params)) {}

ControlResult APMAttitudeController::run(const VehicleState& state,
                                         const Reference& refs,
                                         [[maybe_unused]] ContextBase* error) {
  // // Convert from centidegrees on public interface to radians
  auto heading_rate = std::clamp(refs.yaw_rate, -params_->slew_yaw_max(),
                                 params_->slew_yaw_max());
  const auto heading_angle = refs.yaw;

  // convert thrust vector and heading to a quaternion attitude
  const Eigen::Quaterniond desired_attitude_quat = refs.state.pose.orientation;

  if (params_->rate_bf_ff_enabled) {
    // calculate the angle error in x and y.
    const Eigen::Vector3d attitude_error =
        apm::ThrustVectorRotationAngles(desired_attitude_quat, attitude_target_)
            .attitude_error;

    // When yaw acceleration limiting is enabled, the yaw input shaper
    // constrains angular acceleration about the yaw axis, slewing the output
    // rate towards the input rate.
    ang_vel_target_.x() = apm::InputShapingAngle(
        attitude_error.x(), params_->input_tc, params_->ang_accel_max.x(),
        ang_vel_target_.x(), params_->dt);
    ang_vel_target_.y() = apm::InputShapingAngle(
        attitude_error.y(), params_->input_tc, params_->ang_accel_max.y(),
        ang_vel_target_.y(), params_->dt);
    ang_vel_target_.z() = apm::InputShapingAngle(
        attitude_error.z(), params_->input_tc, params_->ang_accel_max.z(),
        ang_vel_target_.z(), heading_rate, params_->ang_vel_max.z(),
        params_->dt);

    // Limit the angular velocity
    ang_vel_target_ =
        apm::BodyRateLimiting(ang_vel_target_, params_->ang_vel_max);
  } else {
    // set persisted quaternion target attitude
    attitude_target_ = desired_attitude_quat;

    // Set rate feedforward requests to zero
    ang_vel_target_.setZero();
  }

  // Call quaternion attitude controller
  auto ang_vel_body =
      attitudeControllerRunQuat(state.pose.orientation, state.twist.angular);

  return {Setpoint{{}, VehicleInput{0.0, ang_vel_body}},
          ControllerErrc::kSuccess};
}

Eigen::Vector3d updateAngVelTargetFromAttError(
    const Eigen::Vector3d& attitude_error_rot_vec_rad);

Eigen::Vector3d APMAttitudeController::attitudeControllerRunQuat(
    const Eigen::Quaterniond& orientation, const Eigen::Vector3d& body_rates) {
  // This represents a quaternion rotation in NED frame to the body

  // This vector represents the angular error to rotate the thrust vector
  // using x and y and heading using z
  const auto& [_, attitude_error, thrust_error_angle] =
      apm::ThrustHeadingRotationAngles(
          attitude_target_, orientation, params_->kp_angle.z(),
          params_->kp_yawrate, params_->ang_accel_max.z());

  // Compute the angular velocity corrections in the body frame from the
  // attitude error
  // ensure angular velocity does not go over configured limits
  Eigen::Vector3d ang_vel_body = updateAngVelTargetFromAttError(attitude_error);
  ang_vel_body = apm::BodyRateLimiting(ang_vel_body, params_->ang_vel_max);

  // rotation from the target frame to the body frame
  const Eigen::Quaterniond rotation_target_to_body =
      orientation.inverse() * attitude_target_;

  // target angle velocity vector in the body frame
  const Eigen::Vector3d ang_vel_body_feedforward =
      rotation_target_to_body * ang_vel_target_;

  constexpr double kThrustErrorAngleThresh = deg2rad(30.0);

  // Correct the thrust vector and smoothly add feedforward and yaw input
  feedforward_scalar_ = 1.0;
  if (thrust_error_angle > kThrustErrorAngleThresh * 2.0) {
    ang_vel_body.z() = body_rates.z();
  } else if (thrust_error_angle > kThrustErrorAngleThresh) {
    feedforward_scalar_ = 1.0 - (thrust_error_angle - kThrustErrorAngleThresh) /
                                    kThrustErrorAngleThresh;
    ang_vel_body.x() += ang_vel_body_feedforward.x() * feedforward_scalar_;
    ang_vel_body.y() += ang_vel_body_feedforward.y() * feedforward_scalar_;
    ang_vel_body.z() += ang_vel_body_feedforward.z();
    ang_vel_body.z() = body_rates.z() * (1.0 - feedforward_scalar_) +
                       ang_vel_body.z() * feedforward_scalar_;
  } else {
    ang_vel_body += ang_vel_body_feedforward;
  }

  if (params_->rate_bf_ff_enabled) {
    // rotate target and normalize
    attitude_target_ *=
        fsc::AngleAxisToQuaternion(ang_vel_target_ * params_->dt);
  }

  return ang_vel_body;
}

Eigen::Vector3d APMAttitudeController::updateAngVelTargetFromAttError(
    const Eigen::Vector3d& attitude_error) const {
  // minimum body-frame acceleration limit for the stability controller (for
  // roll and pitch axis)
  constexpr double kMinRollPitchAccel = deg2rad(40.0);

  // maximum body-frame acceleration limit for the stability controller (for
  // roll and pitch axis)
  constexpr double kMaxRollPitchAccel = deg2rad(720.0);
  // minimum body-frame acceleration limit for the stability controller (for
  // yaw axis)
  constexpr double kMinYawAccel = deg2rad(10.0);

  // maximum body-frame acceleration limit for the stability controller (for
  // yaw axis)
  constexpr double kMaxYawAccel = deg2rad(120.0);
  if (!params_->use_sqrt_controller) {
    return params_->kp_angle.cwiseProduct(attitude_error);
  }
  Eigen::Vector3d body_rate_target;
  // Compute the roll angular velocity demand from the roll angle error
  if (params_->ang_accel_max.x() > 0.0) {
    body_rate_target.x() = apm::PiecewiseProportionalSqrt(
        attitude_error.x(), params_->kp_angle.x(),
        std::clamp(params_->ang_accel_max.x() / 2.0, kMinRollPitchAccel,
                   kMaxRollPitchAccel),
        params_->dt);
  } else {
    body_rate_target.x() = params_->kp_angle.x() * attitude_error.x();
  }

  // Compute the pitch angular velocity demand from the pitch angle error
  if (params_->ang_accel_max.y() > 0.0) {
    body_rate_target.y() = apm::PiecewiseProportionalSqrt(
        attitude_error.y(), params_->kp_angle.y(),
        std::clamp(params_->ang_accel_max.y() / 2.0, kMinRollPitchAccel,
                   kMaxRollPitchAccel),
        params_->dt);
  } else {
    body_rate_target.y() = params_->kp_angle.y() * attitude_error.y();
  }

  // Compute the yaw angular velocity demand from the yaw angle error
  if (params_->ang_accel_max.z() > 0.0) {
    body_rate_target.z() = apm::PiecewiseProportionalSqrt(
        attitude_error.z(), params_->kp_angle.z(),
        std::clamp(params_->ang_accel_max.z() / 2.0, kMinYawAccel,
                   kMaxYawAccel),
        params_->dt);
  } else {
    body_rate_target.z() = params_->kp_angle.z() * attitude_error.z();
  }

  return body_rate_target;
}
bool APMAttitudeControllerParams::load(const ParameterLoaderBase& loader,
                                       LoggerBase* logger) {
  std::ignore = loader.getParam("dt", dt);
  if (dt == -1.0) {
    logger->log(
        Severity::kWarning,
        "`dt` left at default. Take care to set it to a positive value later");
  } else if (dt < 0.0) {
    logger->log(Severity::kError, "`dt` must not be negative");
    return true;
  }

  std::ignore = loader.getParam("input_tc", input_tc);
  if (input_tc < 0.0) {
    logger->log(Severity::kError, "`input_tc` must not be negative");
    return true;
  }
  std::ignore = loader.getParam("use_sqrt_controller", use_sqrt_controller);
  std::ignore = loader.getParam("enable_rate_feedforward", rate_bf_ff_enabled);
  std::ignore = loader.getParam("roll_p", kp_angle.x());
  std::ignore = loader.getParam("pitch_p", kp_angle.y());
  std::ignore = loader.getParam("yaw_p", kp_angle.z());
  std::ignore = loader.getParam("yawrate_p", kp_yawrate);
  std::ignore = loader.getParam("rollrate_max", ang_vel_max.x());
  std::ignore = loader.getParam("pitchrate_max", ang_vel_max.y());
  std::ignore = loader.getParam("yawrate_max", ang_vel_max.z());
  std::ignore = loader.getParam("roll_accel_max", ang_accel_max.x());
  std::ignore = loader.getParam("pitch_accel_max", ang_accel_max.y());
  std::ignore = loader.getParam("yaw_accel_max", ang_accel_max.z());
  return true;
}

std::string APMAttitudeControllerParams::toString() const {
  const Eigen::IOFormat f{
      Eigen::StreamPrecision, 0, ",", ";\n", "", "", "[", "]"};

  std::ostringstream oss;

  oss << "APM Attitude Controller parameters:" << std::boolalpha     //
      << "\nrate_bf_ff_enabled: " << rate_bf_ff_enabled              //
      << "\nuse_sqrt_controller: " << use_sqrt_controller            //
      << "\ninput_tc: " << input_tc                                  //
      << "\ndt: " << dt                                              //
      << "\nkp_angle: " << kp_angle.transpose().format(f)            //
      << "\nang_accel_lax: " << ang_accel_max.transpose().format(f)  //
      << "\nang_vel_lax: " << ang_vel_max.transpose().format(f);     //
  return oss.str();
}
}  // namespace fsc
