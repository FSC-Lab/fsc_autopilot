cmake_minimum_required(VERSION 3.24.0)
project(fsc_autopilot_ros VERSION 0.4.0)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(fsc_autopilot REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(fsc_autopilot_msgs REQUIRED)

set(_ob_deps
    ament_cmake
    fsc_autopilot
    rclcpp
    tf2_ros
    tf2_eigen
    geometry_msgs
    std_msgs
    std_srvs
    mavros_msgs
    fsc_autopilot_msgs
   )

# export
ament_export_dependencies(${_ob_deps})

# add ros_support.cpp
add_library(ros_support src/ros_support.cpp)
target_link_libraries(ros_support
  ${cpp_typesupport_target} 
)
target_include_directories(ros_support PUBLIC include/)
ament_target_dependencies(ros_support rclcpp fsc_autopilot)

# add autopilot_client.cpp
add_library(autopilot_client src/autopilot_client.cpp)
target_link_libraries(autopilot_client
  ${cpp_typesupport_target} 
)
target_include_directories(autopilot_client PUBLIC include/)
ament_target_dependencies(autopilot_client rclcpp fsc_autopilot mavros_msgs tf2_ros tf2_eigen sensor_msgs)

add_executable(autopilot_node src/autopilot_node.cpp)
target_link_libraries(autopilot_node
  ${cpp_typesupport_target} 
)
target_link_libraries(autopilot_node PRIVATE ${PROJECT_NAME})
ament_target_dependencies(autopilot_client rclcpp fsc_autopilot_ros)

set(_ob_list
  ros_support
  autopilot_client
  autopilot_node
)
install(TARGETS ${_ob_list} DESTINATION lib/${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
