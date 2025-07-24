cmake_minimum_required(VERSION 3.24.0)
project(fsc_autopilot_msgs)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(message_generation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

set(_ob_deps
    ament_cmake
    message_generation
    std_msgs
    geometry_msgs
   )

# export
ament_export_dependencies(${_ob_deps})

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionControllerReference.msg"
  "msg/PositionControllerState.msg"
  "msg/AttitudeControllerState.msg"
  "msg/UDEState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
