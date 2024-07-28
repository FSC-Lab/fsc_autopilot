cmake_minimum_required(VERSION 3.20.0)
project(fsc_autopilot_ros2)

find_package(fsc_autopilot REQUIRED)
find_package(fsc_autopilot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_library(${PROJECT_NAME} src/ros2_support.cpp src/autopilot_client.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp
  tf2_eigen
  geometry_msgs
  mavros_msgs
  nav_msgs
  sensor_msgs
  fsc_autopilot_msgs)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES} fsc_autopilot::core
         fsc_autopilot::position_control fsc_autopilot::attitude_control)

add_executable(autopilot_node src/autopilot_node.cpp)
target_link_libraries(autopilot_node PRIVATE ${PROJECT_NAME})

ament_package()
