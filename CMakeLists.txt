cmake_minimum_required(VERSION 3.14.0)
project(fsc_autopilot_full VERSION 0.2.0)

# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;--header-filter=^${sourceDir}/include")

add_subdirectory(fsc_autopilot)
add_subdirectory(fsc_autopilot_ros)
