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
#include <vector>

#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/core/vehicle_model.hpp"
#include "fsc_autopilot/math/low_pass_filter.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot_ros/ros_support.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/transform_datatypes.h"

namespace nodelib {

class AutopilotClient {
 public:
  explicit AutopilotClient(rclcpp::Node::SharedPtr node);

 private:
  void setupPubSub(const std::string& uav_prefix);

  void outerLoop();

  void innerLoop();

  void watchdog();

  bool loadParams();

  bool initialized_{false};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<fsc::PositionControllerBase> pos_ctrl_;
  std::unique_ptr<fsc::UDEBase> ude_;
  std::unique_ptr<fsc::AttitudeControllerBase> att_ctrl_;

  fsc::VehicleModel mdl_;
  fsc::VehicleState state_;

  tf2::Stamped<fsc::PositionControllerReference> outer_ref_;
  fsc::AttitudeReference inner_ref_;
  fsc::VehicleInput input_;

  rclcpp::Time last_odom_timestamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_timestamp_{0, 0, RCL_ROS_TIME};

  // ROS 2 subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<fsc_autopilot_msgs::msg::PositionControllerReference>::SharedPtr pos_ref_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  // ROS 2 publishers
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<fsc_autopilot_msgs::msg::AttitudeControllerState>::SharedPtr attitude_error_pub_;
  rclcpp::Publisher<fsc_autopilot_msgs::msg::PositionControllerState>::SharedPtr tracking_error_pub_;
  rclcpp::Publisher<fsc_autopilot_msgs::msg::UDEState>::SharedPtr ude_state_pub_;

  mavros_msgs::msg::State vehicle_state_;

  mavros_msgs::msg::AttitudeTarget cmd_;

  fsc::BatchLowPassFilter<Eigen::Vector3d> imu_filter_;
  double outer_period_;
  double inner_period_;
  rclcpp::TimerBase::SharedPtr outer_loop_;
  rclcpp::TimerBase::SharedPtr inner_loop_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  bool enable_inner_controller_{false};  // flag indicating whether inner attitude controller is on

  RosLogger logger_;
};

}  

#endif  