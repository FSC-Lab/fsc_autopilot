cmake_minimum_required(VERSION 3.20.0)
project(fsc_autopilot_ros2)

find_package(fsc_autopilot REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS ros2cpp
             tf2_eigen
             geometry_msgs
             std_msgs
             std_srvs
             message_generation
             dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/TrackingControl.cfg)

add_message_files(DIRECTORY msg FILES TrackingReference.msg TrackingError.msg
                  UDEState.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)

add_library(${PROJECT_NAME} src/ros2_support.cpp src/autopilot_client.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES} fsc_autopilot::core
         fsc_autopilot::position_control fsc_autopilot::attitude_control)

add_executable(autopilot_node src/autopilot_node.cpp)
target_link_libraries(autopilot_node PRIVATE ${PROJECT_NAME})
