cmake_minimum_required(VERSION 3.14.0)
project(tracking_control)

# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;--header-filter=^${sourceDir}/include")
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_eigen geometry_msgs std_msgs
                                        std_srvs message_generation)

add_message_files(DIRECTORY msg FILES TrackingReference.msg Tracking.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(utils src/utils.cpp)
add_library(tracking_control_controllers src/tracking_controller.cpp
                                         src/nonlinear_geometric_controller.cpp)

target_include_directories(tracking_control_controllers PUBLIC include)
target_compile_features(tracking_control_controllers PUBLIC cxx_std_17)
target_link_libraries(tracking_control_controllers PUBLIC Eigen3::Eigen utils)

add_library(tracking_control::controllers ALIAS tracking_control_controllers)
set_target_properties(tracking_control_controllers PROPERTIES EXPORT_NAME
                                                              controllers)

add_library(tracking_control_client src/tracking_control_client.cpp)
add_dependencies(tracking_control_client ${PROJECT_NAME}_generate_messages_cpp
                 ${PROJECT_NAME}_gencpp)
target_include_directories(tracking_control_client
                           PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  tracking_control_client PUBLIC tracking_control::controllers fmt::fmt
                                 ${catkin_LIBRARIES})

add_executable(tracking_control_node src/tracking_control_node.cpp)

target_link_libraries(tracking_control_node PRIVATE tracking_control_client
                                                    utils ${catkin_LIBRARIES})

add_dependencies(tracking_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
