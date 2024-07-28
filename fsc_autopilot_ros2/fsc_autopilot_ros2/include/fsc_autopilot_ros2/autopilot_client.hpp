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

#ifndef INCLUDE_FSC_AUTOPILOT_ROS2_AUTOPILOT_CLIENT_HPP_
#define INCLUDE_FSC_AUTOPILOT_ROS2_AUTOPILOT_CLIENT_HPP_

#include <memory>
#include <rclcpp/timer.hpp>
#include <string>
#include <unordered_map>

#include "fsc_autopilot/attitude_control/apm_attitude_controller.hpp"
#include "fsc_autopilot/math/polynomial.hpp"
#include "fsc_autopilot/position_control/tracking_controller.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot_msgs/msg/tracking_error.hpp"
#include "fsc_autopilot_msgs/msg/tracking_reference.hpp"
#include "fsc_autopilot_ros2/ros2_support.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/imu.hpp"

namespace nodelib {

class TrackingControlClient : public rclcpp::Node {
 public:
  using AttitudeController = fsc::APMAttitudeController;
  using TrackingController = fsc::TrackingController;

  using ControllerSharedPtr = std::shared_ptr<fsc::ControllerBase>;
  TrackingControlClient();

 private:
  using MotorCurveType = math::Polynomial<double>;

  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void setpointCb(
      const fsc_autopilot_msgs::msg::TrackingReference::ConstSharedPtr& msg);
  void mavrosStateCb(const mavros_msgs::msg::State& msg);

  void outerLoop();

  void innerLoop();

  void watchdog();

  bool loadParams();
  void setupRosTopics();

  bool initialized_{false};
  bool check_reconfiguration_{true};
  TrackingController tracking_ctrl_;
  AttitudeController att_ctrl_;

  fsc::VehicleState state_;

  fsc::Reference outer_ref_;
  fsc::Reference inner_ref_;

  fsc::APMAttitudeControllerParams ac_params_;
  fsc::TrackingControllerParameters tc_params_;
  rclcpp::Time odom_last_recv_time_;
  rclcpp::Time imu_last_recv_time_;
  rclcpp::Time state_last_recv_time_;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      attitude_error_pub_;
  rclcpp::Publisher<fsc_autopilot_msgs::msg::TrackingError>::SharedPtr
      tracking_error_pub_;

  mavros_msgs::msg::State mavrosState_;

  mavros_msgs::msg::AttitudeTarget cmd_;

  MotorCurveType motor_curve_;

  rclcpp::Time outer_loop_event_time_;
  rclcpp::TimerBase::SharedPtr outer_loop_;
  rclcpp::Time inner_loop_event_time_;
  rclcpp::TimerBase::SharedPtr inner_loop_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  bool enable_inner_controller_{
      false};  // flag indicating wether inner atttiude controller is on

  RosLogger logger_{*this};
};

}  // namespace nodelib

#endif  // INCLUDE_FSC_AUTOPILOT_ROS2_AUTOPILOT_CLIENT_HPP_
