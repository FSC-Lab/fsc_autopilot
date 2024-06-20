#ifndef TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_

#include <string>
#include <unordered_map>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "tracking_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/polynomial.hpp"
#include "tracking_control/tracking_controller.hpp"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "mavros_msgs/State.h"

namespace nodelib {
class TrackingControlClient {
public:
  using AttitudeController = control::NonlinearGeometricController<double>;
  using TrackingController = control::TrackingController<double>;
  TrackingControlClient();

private:
  using MotorCurveType = math::Polynomial<double>;
  void poseCb(const geometry_msgs::PoseStampedConstPtr &msg);

  void twistCb(const geometry_msgs::TwistStampedConstPtr &msg);
  void imuCb(const sensor_msgs::ImuConstPtr &msg);
  void setpointCb(const trajectory_msgs::JointTrajectoryPointConstPtr &msg);
  void mavrosStateCb(const mavros_msgs::State &msg);

  void mainLoop(const ros::TimerEvent &event);

  void dispPara(void);
  void initVariables(void);
  void loadParams(void);
  void setupRosTopics(void);

  // get time diff from syste
  // convert it to double with unit (s)
  double getTimeDiff(const ros::Time& currTime, const ros::Time& lastTime);

  ros::NodeHandle nh_;
  TrackingController tracking_ctrl_;
  AttitudeController att_ctrl_;

  TrackingController::State state_;

  AttitudeController::State att_ctrl_state_{
      TrackingController::QuaternionType::Identity()};

  TrackingController::Reference refs_;

  AttitudeController::Parameters ac_params_;
  TrackingController::Parameters tc_params_;
  std::unordered_map<std::string, ros::Subscriber> subs_;
  ros::Publisher setpoint_pub_;
  ros::Publisher setpoint_pos_error_pub_;
  ros::Publisher setpoint_vel_error_pub_;
  ros::Publisher setpoint_attitude_error_pub_;
  ros::Publisher acc_setpoint_pub_;
  ros::Publisher ude_estimate_pub_;
  ros::Publisher debug_data_pub_;
  mavros_msgs::State mavrosState_;

  MotorCurveType motor_curve_;

  double ros_rate_{0.0};
  ros::Timer timer_;
  bool enable_inner_controller_{false}; // flag indicating wether inner atttiude controller is on
  double timeStep{0.0}; // stores the time difference between to consecutive calls
  ros::Time currTime;
  ros::Time lastTime;
};

} // namespace nodelib

#endif // TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
