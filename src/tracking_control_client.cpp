#include "tracking_control/tracking_control_client.hpp"

#include "fmt/core.h"
#include "fmt/ostream.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tracking_control/Tracking.h"
#include "tracking_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/tracking_controller.hpp"

namespace nodelib {
using namespace std::string_literals;  // NOLINT

void validateAndSetVector(std::string_view name, const std::vector<double>& src,
                          Eigen::Ref<Eigen::VectorXd> dst) {
  const auto dst_sz = dst.size();
  const auto src_sz = src.size();
  if (src_sz != 3) {
    ROS_ERROR_STREAM(fmt::format("{} must be a length-{} array; Got size {}",
                                 name, dst_sz, src_sz));
    return;
  }
  dst = Eigen::VectorXd::Map(src.data(), static_cast<Eigen::Index>(src.size()));
}

TrackingControlClient::TrackingControlClient() {
  state_.ctx = std::make_shared<fsc::TrackingControllerContext>();

  ros::NodeHandle pnh("~");
  loadParams();
  ros_rate_ = pnh.param("tracking_controller/rate", 30);
  const ros::Rate update_rate(ros_rate_);

  const auto mavros_ns = pnh.param("mavros_ns", "/mavros"s);

  subs_.emplace(
      "pose"s,
      nh_.subscribe(fmt::format("{}/local_position/adjusted", mavros_ns), 1,
                    &TrackingControlClient::poseCb, this));
  subs_.emplace(
      "twist"s,
      nh_.subscribe(fmt::format("{}/local_position/velocity_local", mavros_ns),
                    1, &TrackingControlClient::twistCb, this));

  subs_.emplace("accel"s,
                nh_.subscribe(fmt::format("{}/imu/data", mavros_ns), 1,
                              &TrackingControlClient::imuCb, this));

  subs_.emplace("target"s,
                nh_.subscribe("tracking_controller/target", 1,
                              &TrackingControlClient::setpointCb, this));

  subs_.emplace("state"s,
                nh_.subscribe(fmt::format("{}/state", mavros_ns), 1,
                              &TrackingControlClient::mavrosStateCb, this));

  setpoint_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      fmt::format("{}/setpoint_raw/attitude", mavros_ns), 1);
  setpoint_pos_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_pos_error", 1);
  setpoint_vel_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_vel_error", 1);
  setpoint_attitude_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/setpoint_attitude_error", 1);
  ude_estimate_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/ude_estimate", 1);
  acc_setpoint_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "tracking_controller/acc_setpoint", 1);
  debug_data_pub_ = nh_.advertise<tracking_control::Tracking>(
      "tracking_controller/output_data", 1);

  // show the parameters
  dispPara();

  // initialize variables
  initVariables();

  // register the main loop callback function here
  timer_ = nh_.createTimer(update_rate, &TrackingControlClient::mainLoop, this);
}

void TrackingControlClient::poseCb(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // the estimation only provides position measurement
  tf2::fromMsg(msg->pose.position, state_.pose.position);
  // tf2::fromMsg(msg->pose.orientation, state_.orientation);
  // tf2::fromMsg(msg->pose.orientation, att_ctrl_state_.attitude);
}

void TrackingControlClient::twistCb(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  tf2::fromMsg(msg->twist.linear, state_.twist.linear);
}

void TrackingControlClient::imuCb(const sensor_msgs::ImuConstPtr& msg) {
  tf2::fromMsg(msg->linear_acceleration, state_.accel.linear);
  tf2::fromMsg(msg->orientation, state_.pose.orientation);
}

void TrackingControlClient::setpointCb(
    const tracking_control::TrackingReferenceConstPtr& msg) {
  tf2::fromMsg(msg->pose.position, refs_.state.pose.position);
  tf2::fromMsg(msg->pose.orientation, refs_.state.pose.orientation);
  tf2::fromMsg(msg->twist.linear, refs_.state.twist.linear);
  tf2::fromMsg(msg->twist.angular, refs_.state.twist.angular);
  tf2::fromMsg(msg->accel.linear, refs_.state.accel.linear);
  tf2::fromMsg(msg->accel.angular, refs_.state.accel.angular);
  refs_.yaw = msg->yaw * M_PI / 180.0;
  // ? normal to the velocity? if 0, will give some incorrect values
  // refs_.yaw = atan2(refs_.velocity.y(), refs_.velocity.x());
  // to do: set this one to zero
}

void TrackingControlClient::mavrosStateCb(const mavros_msgs::State& msg) {
  mavrosState_ = msg;
}

void TrackingControlClient::mainLoop(const ros::TimerEvent& event) {
  // calculate time step
  currTime = event.current_real;
  timeStep = getTimeDiff(currTime, lastTime);
  lastTime = currTime;

  // check drone status
  bool int_flag = (mavrosState_.connected != 0U) &&
                  (mavrosState_.armed != 0U) &&
                  (mavrosState_.mode == "OFFBOARD");

  auto ctx = std::make_unique<fsc::TrackingControllerContext>();
  if (!ctx->set("dt", timeStep)) {
    ROS_ERROR("Failed to set timestep");
  }
  if (!ctx->set("interrupt_ude", int_flag)) {
    ROS_ERROR("Failed to set interrupt_ude flag");
  }
  state_.ctx = std::move(ctx);

  // outerloop control
  const auto& [outer_success, pos_ctrl_out, p_pos_ctrl_err, msg] =
      tracking_ctrl_.run(state_, refs_);

  if (p_pos_ctrl_err->name() != "tracking_controller.error") {
    ROS_FATAL(
        "Expected name of error struct to be `tracking_controller.error`; Got "
        "%s",
        p_pos_ctrl_err->name().c_str());
    return;
  }

  const auto& pos_ctrl_err =
      *std::static_pointer_cast<fsc::TrackingControllerError>(p_pos_ctrl_err);

  if (!outer_success) {
    ROS_ERROR("Outer controller failed!: %s", msg.c_str());
    return;
  }

  // define output messages
  mavros_msgs::AttitudeTarget pld;
  tracking_control::Tracking pld_debug_data;
  pld.header.stamp = event.current_real;
  // convert thrust command to normalized thrust input
  // if the thrust is an acc command, then the thrust curve must change as well
  pld.thrust = std::clamp(
      static_cast<float>(motor_curve_.vals(pos_ctrl_out.input.thrust)), 0.0F,
      1.0F);

  // std::cout<<"normalized thrust is: "<<pld.thrust<<'\n';

  geometry_msgs::Vector3Stamped pld_pos_err;
  pld_pos_err.header.stamp = event.current_real;
  tf2::toMsg(pos_ctrl_err.position_error, pld_pos_err.vector);

  geometry_msgs::Vector3Stamped pld_vel_err;
  pld_vel_err.header.stamp = event.current_real;
  tf2::toMsg(pos_ctrl_err.velocity_error, pld_vel_err.vector);

  geometry_msgs::Vector3Stamped pld_att_err;
  pld_att_err.header.stamp = event.current_real;

  geometry_msgs::Vector3Stamped ude_estimate;
  ude_estimate.header.stamp = event.current_real;

  geometry_msgs::Vector3Stamped accsp;

  if (enable_inner_controller_) {
    ROS_DEBUG("Running inner controller");
    fsc::Reference inner_ref;
    inner_ref.state.pose.orientation = pos_ctrl_out.state.pose.orientation;
    const auto& [inner_success, att_ctrl_out, p_att_ctrl_err, _] =
        att_ctrl_.run(state_, inner_ref);

    pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    if (p_att_ctrl_err->name() != "nonlinear_geometric_controller.error") {
      ROS_FATAL(
          "Expected name of error struct to be "
          "`nonlinear_geometric_controller.error`; "
          "Got "
          "%s",
          p_att_ctrl_err->name().c_str());
    }
    const auto& att_ctrl_err =
        *std::static_pointer_cast<fsc::NonlinearGeometricControllerError>(
            p_att_ctrl_err);

    pld_att_err.vector.x = att_ctrl_err.attitude_error.x() * 180.0 / M_PI;
    pld_att_err.vector.y = att_ctrl_err.attitude_error.y() * 180.0 / M_PI;
    pld_att_err.vector.z = att_ctrl_err.attitude_error.z() * 180.0 / M_PI;

    tf2::toMsg(std::get<Eigen::Vector3d>(att_ctrl_out.input.command),
               pld.body_rate);
  } else {
    pld.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE &
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE &
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    pld.orientation =
        tf2::toMsg(std::get<Eigen::Quaterniond>(pos_ctrl_out.input.command));

    // update attitude error to be 0 (because inner controller is disabled)
    pld_att_err.vector.x = 0.0;
    pld_att_err.vector.y = 0.0;
    pld_att_err.vector.z = 0.0;
  }

  // update debug output
  pld_debug_data.intFlag = pos_ctrl_err.intFlag;
  pld_debug_data.altiThreshold = pos_ctrl_err.altiThreshold;
  pld_debug_data.thrustSetpoint = pos_ctrl_err.thrust_sp;
  pld_debug_data.thrustPerRotor = pos_ctrl_err.thrustPerRotor;
  pld_debug_data.expectedThrust.x = pos_ctrl_err.expectedThrust.x();
  pld_debug_data.expectedThrust.y = pos_ctrl_err.expectedThrust.y();
  pld_debug_data.expectedThrust.z = pos_ctrl_err.expectedThrust.z();
  pld_debug_data.disturbanceEstimate.x = pos_ctrl_err.disturbanceEstimate.x();
  pld_debug_data.disturbanceEstimate.y = pos_ctrl_err.disturbanceEstimate.y();
  pld_debug_data.disturbanceEstimate.z = pos_ctrl_err.disturbanceEstimate.z();
  pld_debug_data.inertialForce.x = pos_ctrl_err.inertialForce.x();
  pld_debug_data.inertialForce.y = pos_ctrl_err.inertialForce.y();
  pld_debug_data.inertialForce.z = pos_ctrl_err.inertialForce.z();

  tf2::toMsg(pos_ctrl_err.ude_output, ude_estimate.vector);
  tf2::toMsg(pos_ctrl_err.accel_sp, accsp.vector);

  setpoint_pub_.publish(pld);
  setpoint_pos_error_pub_.publish(pld_pos_err);
  setpoint_vel_error_pub_.publish(pld_vel_err);
  setpoint_attitude_error_pub_.publish(pld_att_err);
  ude_estimate_pub_.publish(ude_estimate);
  acc_setpoint_pub_.publish(accsp);
  debug_data_pub_.publish(pld_debug_data);
}

void TrackingControlClient::dispPara() {
  // offboard control mode
  if (enable_inner_controller_) {
    ROS_INFO("Inner controller (rate mode) enabled!");
  } else {
    ROS_INFO("Inner controller bypassed! (attitude setpoint mode)");
  }
  // drone parameters
  ROS_INFO("Quadrotor mass: %f", tc_params_.vehicle_mass);
  // control parameters
  ROS_INFO("Tracking controller gains:");
  ROS_INFO("k_pos/x: %f, k_pos/y: %f, k_pos/z: %f", tc_params_.k_pos[0],
           tc_params_.k_pos[1], tc_params_.k_pos[2]);
  ROS_INFO("k_vel/x: %f, k_vel/y: %f, k_vel/z: %f", tc_params_.k_vel[0],
           tc_params_.k_vel[1], tc_params_.k_vel[2]);

  // controller constraints

  ROS_INFO("height_threshold: %f", tc_params_.de_height_threshold);

  // update rate
  ROS_INFO("update rate: %f", ros_rate_);

  // ude parameters
  ROS_INFO("de_gain: %f", tc_params_.de_gain);
  ROS_INFO("de_height_threshold: %f", tc_params_.de_height_threshold);
}

void TrackingControlClient::loadParams() {
  ros::NodeHandle pnh("~");
  // put load parameters into a function
  enable_inner_controller_ =
      pnh.param("tracking_controller/enable_inner_controller", false);
  ROS_INFO("Inner controller is %s",
           (enable_inner_controller_ ? "enabled" : "disabled"));

  constexpr auto kDefaultKPosXY = 1.0;
  constexpr auto kDefaultKPosZ = 10.0;
  tc_params_.k_pos <<  // Force a line break
      pnh.param("tracking_controller/k_pos/x", kDefaultKPosXY),
      pnh.param("tracking_controller/k_pos/y", kDefaultKPosXY),
      pnh.param("tracking_controller/k_pos/z", kDefaultKPosZ);

  constexpr auto kDefaultKVelXY = 1.5;
  constexpr auto kDefaultkVelZ = 3.3;
  tc_params_.k_vel <<  // Force a line break
      pnh.param("tracking_controller/k_vel/x", kDefaultKVelXY),
      pnh.param("tracking_controller/k_vel/y", kDefaultKVelXY),
      pnh.param("tracking_controller/k_vel/z", kDefaultkVelZ);

  tc_params_.de_lb << pnh.param("tracking_controller/de/lbx",
                                tc_params_.de_lb.x()),
      pnh.param("tracking_controller/de/lby", tc_params_.de_lb.y()),
      pnh.param("tracking_controller/de/lbz", tc_params_.de_lb.z());

  tc_params_.de_ub << pnh.param("tracking_controller/de/lbx",
                                tc_params_.de_ub.x()),
      pnh.param("tracking_controller/de/lby", tc_params_.de_ub.y()),
      pnh.param("tracking_controller/de/lbz", tc_params_.de_ub.z());

  tc_params_.de_height_threshold =
      pnh.param("tracking_controller/de/height_threshold",
                tc_params_.de_height_threshold);
  tc_params_.de_gain =
      pnh.param("tracking_controller/de/gain", tc_params_.de_gain);

  pnh.getParam("tracking_controller/de/is_velocity_based",
               tc_params_.ude_is_velocity_based);

  ROS_INFO_STREAM("UDE is velocity based " << std::boolalpha
                                           << tc_params_.ude_is_velocity_based);

  if (!pnh.getParam("tracking_controller/vehicle_mass",
                    tc_params_.vehicle_mass)) {
    ROS_FATAL("Vehicle mass unspecified in parameters!");
    std::terminate();
  }

  constexpr auto kDefaultAttitudeControllerTau = 0.1;
  ac_params_.time_constant =
      pnh.param("attitude_controller/tau", kDefaultAttitudeControllerTau);

  att_ctrl_.params() = ac_params_;

  tc_params_.min_z_accel = pnh.param("tracking_controller/min_acc", 0.0);

  constexpr auto kDefaultMaxAcceleration = 20.0;
  tc_params_.max_z_accel =
      pnh.param("tracking_controller/max_acc", kDefaultMaxAcceleration);

  constexpr auto kDefaultMaxTiltAngle = 45.0;
  tc_params_.max_tilt_angle =
      pnh.param("tracking_controller/max_tilt_angle", kDefaultMaxTiltAngle);

  tracking_ctrl_.params() = tc_params_;

  const auto mc_coeffs = pnh.param("tracking_controller/motor_curve",
                                   std::vector<double>{0.1, 0.05});
  motor_curve_ = MotorCurveType(Eigen::VectorXd::Map(
      mc_coeffs.data(), static_cast<Eigen::Index>(mc_coeffs.size())));
}

void TrackingControlClient::initVariables() {
  /*
   * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is
   * called 'secs') stamp.nsec: nanoseconds since stamp_secs (in Python the
   * variable is called 'nsecs')
   */
  currTime = ros::Time::now();
  lastTime = ros::Time::now();
}

double TrackingControlClient::getTimeDiff(const ros::Time& currTime,
                                          const ros::Time& lastTime) {
  ros::Duration diff = currTime - lastTime;
  return static_cast<double>(diff.toSec());
}

}  // namespace nodelib
