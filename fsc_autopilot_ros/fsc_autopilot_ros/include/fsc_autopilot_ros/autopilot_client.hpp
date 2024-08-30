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

#include <std_msgs/Int64.h>

#include <memory>
#include <string>
#include <vector>

#include "dynamic_reconfigure/server.h"
#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/core/vehicle_model.hpp"
#include "fsc_autopilot/math/low_pass_filter.hpp"
#include "fsc_autopilot/position_control/lqg_controller.hpp"
#include "fsc_autopilot/position_control/pid_controller.hpp"
#include "fsc_autopilot/position_control/position_controller_base.hpp"
#include "fsc_autopilot/position_control/tracking_controller.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"
#include "fsc_autopilot_ros/TrackingControlConfig.h"
#include "fsc_autopilot_ros/ros_support.hpp"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/State.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "tf2/transform_datatypes.h"

namespace nodelib {

class AutopilotClient {
 public:
  using ControllerSharedPtr = std::shared_ptr<fsc::ControllerBase>;
  AutopilotClient();

 private:
  void setupPubSub(const std::string& uav_prefix);

  void dynamicReconfigureCb(
      const fsc_autopilot_ros::TrackingControlConfig& config,
      std::uint32_t level);

  void outerLoop(const ros::TimerEvent& event);

  void innerLoop(const ros::TimerEvent& event);

  void watchdog(const ros::TimerEvent& event);

  bool loadParams();

  bool initialized_{false};
  ros::NodeHandle nh_;
  LQGController lqg_ctrl_;
  PIDController pid_ctrl_;
  TrackingController tracking_ctrl_;
  std::unique_ptr<fsc::UDEBase> ude_;
  std::unique_ptr<fsc::AttitudeControllerBase> att_ctrl_;

  Eigen::Matrix<double, 6, 1> estimated_state_;

  fsc::VehicleModel mdl_;
  fsc::VehicleState state_;

  tf2::Stamped<fsc::PositionControllerReference> outer_ref_;
  fsc::AttitudeReference inner_ref_;
  fsc::VehicleInput input_;

  ros::Time last_odom_timestamp_{0.0};
  ros::Time last_imu_timestamp_{0.0};

  // Only need to save the Subscribers to keep them alive => stuff them all
  // without distinguishment in a vector
  std::vector<ros::Subscriber> subs_;
  ros::Publisher setpoint_pub_;
  ros::Publisher attitude_error_pub_;
  ros::Publisher tracking_error_pub_;
  ros::Publisher ude_state_pub_;
  ros::Publisher estimated_state_pub_;
  dynamic_reconfigure::Server<fsc_autopilot_ros::TrackingControlConfig>
      cfg_srv_;

  mavros_msgs::State vehicle_state_;

  mavros_msgs::AttitudeTarget cmd_;

  long int controllerOption;

  int64_t previousControllerOption =
      -1;                 // to store the previous controller option
  bool switched = false;  // to store the switch status

  fsc::BatchLowPassFilter<Eigen::Vector3d> imu_filter_;
  double outer_period_;
  double inner_period_;
  ros::Timer outer_loop_;
  ros::Timer inner_loop_;
  bool enable_inner_controller_{
      false};  // flag indicating wether inner atttiude controller is on

  RosLogger logger_{"fsc_autopilot_ros"};
};

}  // namespace nodelib

#endif  // FSC_AUTOPILOT_ROS_AUTOPILOT_CLIENT_HPP_
