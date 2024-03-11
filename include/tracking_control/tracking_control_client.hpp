#ifndef TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_

#include <string>
#include <unordered_map>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "tracking_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/polynomial.hpp"
#include "tracking_control/tracking_controller.hpp"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace nodelib {
class TrackingControlClient {
 public:
  using AttitudeController = control::NonlinearGeometricController<double>;
  using TrackingController = control::TrackingController<double>;
  TrackingControlClient();

 private:
  using MotorCurveType = math::Polynomial<double>;
  void poseCb(const geometry_msgs::PoseStampedConstPtr& msg);

  void twistCb(const geometry_msgs::TwistStampedConstPtr& msg);

  void setpointCb(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

  void mainLoop(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
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
  std::unordered_map<std::string, ros::Subscriber> subs_;
  ros::Publisher setpoint_pub_;

  MotorCurveType motor_curve_;

  ros::Timer timer_;
};

}  // namespace nodelib

#endif  // TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
