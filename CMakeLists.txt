cmake_minimum_required(VERSION 3.18.0)
project(tracking_control)

# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;--header-filter=^${sourceDir}/include")
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

find_package(catkin REQUIRED roscpp tf2_eigen geometry_msgs)

catkin_package()

add_library(tracking_control_client src/tracking_control_client.cpp)
target_compile_features(tracking_control_client PUBLIC cxx_std_17)
target_include_directories(tracking_control_client
                           PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(tracking_control_client PUBLIC Eigen3::Eigen fmt::fmt
                                                     ${catkin_LIBRARIES})

add_executable(tracking_control_node src/tracking_control_node.cpp)
target_link_libraries(tracking_control_node PRIVATE tracking_control_client)
