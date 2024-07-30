cmake_minimum_required(VERSION 3.20.0)
project(fsc_autopilot VERSION 0.3.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(MAIN_PROJECT_NAME ${PROJECT_NAME})
add_subdirectory(modules)

set(PROJECT_INCLUDE_DIRS
    modules/attitude_control/include/ modules/core/include/
    modules/math/include/ modules/position_control/include/
    modules/ude/include/)

set(PROJECT_TARGETS
    ${MAIN_PROJECT_NAME}_core ${MAIN_PROJECT_NAME}_math
    ${MAIN_PROJECT_NAME}_position_control ${MAIN_PROJECT_NAME}_attitude_control
    ${MAIN_PROJECT_NAME}_ude)
include(cmake/install_rules.cmake)
