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

#ifndef FSC_AUTOPILOT_ROS_AUTOPILOT_CLIENT_HPP_
#define FSC_AUTOPILOT_ROS_AUTOPILOT_CLIENT_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "dynamic_reconfigure/server.h"
#include "fsc_autopilot/attitude_control/apm_attitude_controller.hpp"
#include "fsc_autopilot/math/polynomial.hpp"
#include "fsc_autopilot/position_control/tracking_controller.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot_ros2/TrackingControlConfig.h"
#include "fsc_autopilot_ros2/TrackingReference.h"
#include "fsc_autopilot_ros2/ros2_support.hpp"
#include "mavros2_msgs/AttitudeTarget.h"
#include "mavros2_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "ros2/forwards.h"
#include "ros2/node_handle.h"
#include "sensor_msgs/Imu.h"

namespace nodelib {

class TrackingControlClient {
 public:
  using AttitudeController = fsc::APMAttitudeController;
  using TrackingController = fsc::TrackingController;

  using ControllerSharedPtr = std::shared_ptr<fsc::ControllerBase>;
  TrackingControlClient();

 private:
  using MotorCurveType = math::Polynomial<double>;

  void odomCb(const nav_msgs::OdometryConstPtr& msg);
  void imuCb(const sensor_msgs::ImuConstPtr& msg);
  void setpointCb(const fsc_autopilot_ros2::TrackingReferenceConstPtr& msg);
  void mavros2StateCb(const mavros2_msgs::State& msg);

  void dynamicReconfigureCb(
      const fsc_autopilot_ros2::TrackingControlConfig& config,
      std::uint32_t level);

  void outerLoop(const ros2::TimerEvent& event);

  void innerLoop(const ros2::TimerEvent& event);

  void watchdog(const ros2::TimerEvent& event);

  bool loadParams();
  void setupRosTopics();

  bool initialized_{false};
  bool check_reconfiguration_{true};
  ros2::NodeHandle nh_;
  TrackingController tracking_ctrl_;
  AttitudeController att_ctrl_;

  fsc::VehicleState state_;

  fsc::Reference outer_ref_;
  fsc::Reference inner_ref_;

  fsc::APMAttitudeControllerParams ac_params_;
  fsc::TrackingControllerParameters tc_params_;
  ros2::Time odom_last_recv_time_;
  ros2::Time imu_last_recv_time_;
  ros2::Time state_last_recv_time_;
  std::unordered_map<std::string, ros2::Subscriber> subs_;
  ros2::Publisher setpoint_pub_;
  ros2::Publisher attitude_error_pub_;
  ros2::Publisher tracking_error_pub_;
  dynamic_reconfigure::Server<fsc_autopilot_ros2::TrackingControlConfig>
      cfg_srv_;

  mavros2_msgs::State mavros2State_;

  mavros2_msgs::AttitudeTarget cmd_;

  MotorCurveType motor_curve_;

  ros2::Timer outer_loop_;
  ros2::Timer inner_loop_;
  ros2::Timer watchdog_;
  bool enable_inner_controller_{
      false};  // flag indicating wether inner atttiude controller is on

  RosLogger logger_{"fsc_autopilot_ros2"};
};

}  // namespace nodelib

#endif  // FSC_AUTOPILOT_ROS_AUTOPILOT_CLIENT_HPP_