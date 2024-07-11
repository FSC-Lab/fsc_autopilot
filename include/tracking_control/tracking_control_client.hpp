#ifndef TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "dynamic_reconfigure/server.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "tracking_control/TrackingControlConfig.h"
#include "tracking_control/TrackingReference.h"
#include "tracking_control/attitude_control/apm_attitude_controller.hpp"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/polynomial.hpp"
#include "tracking_control/ros_support.hpp"
#include "tracking_control/tracking_controller.hpp"
#include "tracking_control/ude/ude_base.hpp"

namespace nodelib {

class TrackingControlClient {
 public:
  using AttitudeController = fsc::APMAttitudeController;
  using TrackingController = fsc::TrackingController;
  TrackingControlClient();

 private:
  using MotorCurveType = math::Polynomial<double>;

  void odomCb(const nav_msgs::OdometryConstPtr& msg);
  void imuCb(const sensor_msgs::ImuConstPtr& msg);
  void setpointCb(const tracking_control::TrackingReferenceConstPtr& msg);
  void mavrosStateCb(const mavros_msgs::State& msg);

  void dynamicReconfigureCb(
      const tracking_control::TrackingControlConfig& config,
      std::uint32_t level);

  void outerLoop(const ros::TimerEvent& event);

  void innerLoop(const ros::TimerEvent& event);

  void watchdog(const ros::TimerEvent& event);

  void dispPara();
  void loadParams();
  void setupRosTopics();

  bool initialized_{false};
  bool check_reconfiguration_{true};
  ros::NodeHandle nh_;
  TrackingController tracking_ctrl_;
  AttitudeController att_ctrl_;

  fsc::VehicleState state_;

  fsc::Reference outer_ref_;
  fsc::Reference inner_ref_;

  AttitudeController::ParametersSharedPtr ac_params_{
      std::make_shared<AttitudeController::Parameters>()};
  fsc::TrackingController::ParametersSharedPtr tc_params_{
      std::make_shared<fsc::TrackingControllerParameters>()};
  fsc::UDEBase::ParametersSharedPtr ude_params_{
      std::make_shared<fsc::UDEBase::Parameters>()};
  ros::Time odom_last_recv_time_;
  ros::Time imu_last_recv_time_;
  ros::Time state_last_recv_time_;
  std::unordered_map<std::string, ros::Subscriber> subs_;
  ros::Publisher setpoint_pub_;
  ros::Publisher setpoint_attitude_error_pub_;
  ros::Publisher tracking_error_pub_;
  dynamic_reconfigure::Server<tracking_control::TrackingControlConfig> cfg_srv_;

  mavros_msgs::State mavrosState_;

  mavros_msgs::AttitudeTarget cmd_;

  MotorCurveType motor_curve_;

  ros::Timer outer_loop_;
  ros::Timer inner_loop_;
  ros::Timer watchdog_;
  bool enable_inner_controller_{
      false};  // flag indicating wether inner atttiude controller is on

  RosLogger logger_{"tracking_control"};
};

}  // namespace nodelib

#endif  // TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
