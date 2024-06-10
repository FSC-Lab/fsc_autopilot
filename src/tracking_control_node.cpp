#include "tracking_control/tracking_control_client.hpp"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracking_control_node");

  nodelib::TrackingControlClient client;

  ros::spin();
}
