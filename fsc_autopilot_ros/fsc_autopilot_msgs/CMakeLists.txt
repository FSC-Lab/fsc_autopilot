cmake_minimum_required(VERSION 3.20.0)
project(fsc_autopilot_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs
                                        geometry_msgs)

add_message_files(DIRECTORY msg FILES TrackingReference.msg TrackingError.msg
                  UDEState.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)
