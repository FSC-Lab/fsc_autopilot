cmake_minimum_required(VERSION 3.20.0)
project(fsc_autopilot_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}_msgs
  msg/TrackingReference.msg
  msg/TrackingError.msg
  msg/UDEState.msg
  DEPENDENCIES
  std_msgs
  geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
