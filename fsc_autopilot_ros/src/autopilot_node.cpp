#include "fsc_autopilot_ros/autopilot_client.hpp"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "fsc_autopilot_ros_node");

  nodelib::TrackingControlClient client;

  ros::spin();
}
