#include "rclcpp/rclcpp.hpp"
#include "tracking_control/tracking_control_client.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (auto node = std::make_shared<nodelib::TrackingControlClient>(
          "tracking_control_node")) {
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
}
