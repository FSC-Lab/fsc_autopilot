cmake_minimum_required(VERSION 3.14.0)
project(tracking_control VERSION 0.2.0)

# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;--header-filter=^${sourceDir}/include")
find_package(Eigen3 REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
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

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(
  tracking_control_ude
  src/ude_base.cpp src/velocity_based_ude.cpp src/accel_based_ude.cpp
  src/ude_factory.cpp src/logging.cpp)
target_include_directories(tracking_control_ude PUBLIC include)
target_compile_features(tracking_control_ude PUBLIC cxx_std_17)
target_link_libraries(tracking_control_ude PUBLIC Eigen3::Eigen)

add_library(tracking_control::ude ALIAS tracking_control_ude)
set_target_properties(tracking_control_ude PROPERTIES EXPORT_NAME ude)

add_library(
  tracking_control_controllers
  src/tracking_controller.cpp src/nonlinear_geometric_controller.cpp
  src/apm_attitude_controller.cpp)

target_include_directories(tracking_control_controllers PUBLIC include)
target_compile_features(tracking_control_controllers PUBLIC cxx_std_17)
target_link_libraries(
  tracking_control_controllers
  PUBLIC Eigen3::Eigen $<LINK_LIBRARY:WHOLE_ARCHIVE,tracking_control::ude>)

add_library(tracking_control::controllers ALIAS tracking_control_controllers)
set_target_properties(tracking_control_controllers PROPERTIES EXPORT_NAME
                                                              controllers)

add_library(tracking_control_client src/tracking_control_client.cpp)
add_dependencies(tracking_control_client ${PROJECT_NAME}_generate_messages_cpp
                 ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
target_include_directories(tracking_control_client
                           PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(tracking_control_client
                      PUBLIC tracking_control::controllers ${catkin_LIBRARIES})

add_executable(tracking_control_node src/tracking_control_node.cpp)

target_link_libraries(tracking_control_node PRIVATE tracking_control_client
                                                    ${catkin_LIBRARIES})

add_dependencies(tracking_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
