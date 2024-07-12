cmake_minimum_required(VERSION 3.20.0)
project(fsc_autopilot)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(MAIN_PROJECT_NAME ${PROJECT_NAME})
add_subdirectory(modules)
