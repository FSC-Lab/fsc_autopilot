#ifndef TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_

#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tracking_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/polynomial.hpp"
#include "tracking_control/tracking_controller.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace nodelib {
using namespace std::string_literals;
class TrackingControlClient : public rclcpp::Node {
 public:
  using AttitudeController = control::NonlinearGeometricController<double>;
  using TrackingController = control::TrackingController<double>;
  explicit TrackingControlClient(const std::string& name = ""s);

 private:
  using MotorCurveType = math::Polynomial<double>;
  void poseCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

  void twistCb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);

  void setpointCb(
      const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr& msg);

  void mainLoop();

  TrackingController tracking_ctrl_;
  AttitudeController att_ctrl_;

  TrackingController::State state_{TrackingController::Vector3Type::Zero(),
                                   TrackingController::Vector3Type::Zero()};

  AttitudeController::State att_ctrl_state_{
      TrackingController::QuaternionType::Identity()};

  TrackingController::Reference refs_{TrackingController::Vector3Type::Zero(),
                                      TrackingController::Vector3Type::Zero(),
                                      TrackingController::Vector3Type::Zero(),
                                      0.0};

  AttitudeController::Parameters ac_params_;
  TrackingController::Parameters tc_params_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
      target_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;

  MotorCurveType motor_curve_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nodelib

#endif  // TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
