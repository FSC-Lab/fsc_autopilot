#ifndef TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
#define TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/State.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "tracking_control/TrackingControlConfig.h"
#include "tracking_control/TrackingReference.h"
#include "tracking_control/nonlinear_geometric_controller.hpp"
#include "tracking_control/polynomial.hpp"
#include "tracking_control/tracking_controller.hpp"

namespace nodelib {
class TrackingControlClient {
 public:
  using AttitudeController = fsc::NonlinearGeometricController;
  using TrackingController = fsc::TrackingController;
  TrackingControlClient();

 private:
  using MotorCurveType = math::Polynomial<double>;
  void poseCb(const geometry_msgs::PoseStampedConstPtr& msg);

  void twistCb(const geometry_msgs::TwistStampedConstPtr& msg);
  void imuCb(const sensor_msgs::ImuConstPtr& msg);
  void setpointCb(const tracking_control::TrackingReferenceConstPtr& msg);
  void mavrosStateCb(const mavros_msgs::State& msg);

  void dynamicReconfigureCb(
      const tracking_control::TrackingControlConfig& config,
      std::uint32_t level);

  void mainLoop(const ros::TimerEvent& event);

  void dispPara();
  void loadParams();
  void setupRosTopics();

  bool initialized_{false};
  ros::NodeHandle nh_;
  TrackingController tracking_ctrl_;
  AttitudeController att_ctrl_;

  fsc::VehicleState state_;

  fsc::Reference refs_;

  AttitudeController::Parameters ac_params_;
  fsc::TrackingController::ParametersSharedPtr tc_params_{
      std::make_shared<fsc::TrackingControllerParameters>()};
  std::unordered_map<std::string, ros::Subscriber> subs_;
  ros::Publisher setpoint_pub_;
  ros::Publisher setpoint_attitude_error_pub_;
  ros::Publisher tracking_error_pub_;
  dynamic_reconfigure::Server<tracking_control::TrackingControlConfig> cfg_srv_;

  mavros_msgs::State mavrosState_;

  MotorCurveType motor_curve_;

  double ros_rate_{0.0};
  ros::Timer timer_;
  bool enable_inner_controller_{
      false};  // flag indicating wether inner atttiude controller is on
};

}  // namespace nodelib

#endif  // TRACKING_CONTROL_TRACKING_CONTROL_CLIENT_HPP_
