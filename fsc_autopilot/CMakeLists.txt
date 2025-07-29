cmake_minimum_required(VERSION 3.22)
project(fsc_autopilot VERSION 0.4.0)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

# Build flags
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(MAIN_PROJECT_NAME ${PROJECT_NAME})

# Add subdirectories (libraries)
add_subdirectory(modules)

# Optional: enable tests
include(CTest)
option(BUILD_TESTING "Toggles building unit tests" ON)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# Header include directories
set(PROJECT_INCLUDE_DIRS
  modules/attitude_control/include/
  modules/core/include/
  modules/math/include/
  modules/position_control/include/
  modules/ude/include/
  modules/utils/include/
)

# Targets defined in subdirectories
set(PROJECT_TARGETS
  ${MAIN_PROJECT_NAME}_core
  ${MAIN_PROJECT_NAME}_math
  ${MAIN_PROJECT_NAME}_position_control
  ${MAIN_PROJECT_NAME}_attitude_control
  ${MAIN_PROJECT_NAME}_ude
  ${MAIN_PROJECT_NAME}_utils
)

# Install targets and headers
include(cmake/install_rules.cmake)

# Export targets and include dirs to downstream packages
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(${PROJECT_INCLUDE_DIRS})

# Export ament dependencies
ament_export_dependencies(eigen3 rclcpp ...)  # Add if needed

ament_package()
