cmake_minimum_required(VERSION 3.20.0)
project(position_control)

# TODO(Hs293Go): Support FetchContent
find_package(Eigen3 REQUIRED)

add_library(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
  src/robust_controller.cpp src/pd_controller.cpp
  src/position_controller_factory.cpp
  src/lqg_controller.cpp src/pid_controller.cpp)
target_include_directories(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_compile_features(${MAIN_PROJECT_NAME}_${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
  PUBLIC Eigen3::Eigen ${MAIN_PROJECT_NAME}::core ${MAIN_PROJECT_NAME}::math)

add_library(${MAIN_PROJECT_NAME}::${PROJECT_NAME} ALIAS
            ${MAIN_PROJECT_NAME}_${PROJECT_NAME})
set_target_properties(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                      PROPERTIES EXPORT_NAME ${PROJECT_NAME})


# cmake_minimum_required(VERSION 3.20.0)
# project(position_control)



# # Find Eigen3
# find_package(Eigen3 REQUIRED)

# # Set LibTorch paths
# list(APPEND CMAKE_PREFIX_PATH "/home/fsc2/catkin_ws/libtorch/share/cmake/Torch")
# find_package(Torch REQUIRED)
# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# # Define your library explicitly with a target name
# add_library(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
#   src/tracking_controller.cpp
#   src/lqg_controller.cpp
#   src/pid_controller.cpp
#   src/dnn_inference.cpp
# )

# # set_property(TARGET ${MAIN_PROJECT_NAME}_${PROJECT_NAME} PROPERTY CXX_STANDARD 17)


# # Convert include directories to absolute paths
# # set(LIBTORCH_INCLUDE_DIRS "/home/fsc2/catkin_ws/libtorch/include")
# # set(LIBTORCH_API_INCLUDE_DIRS "/home/fsc2/catkin_ws/libtorch/include/torch/csrc/api/include")
# # set(LIBTORCH_LIBRARIES "/home/fsc2/catkin_ws/libtorch/lib")



# # Include directories
# target_include_directories(
#   ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
#   PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
#          $<INSTALL_INTERFACE:include>
#         #  ${LIBTORCH_INCLUDE_DIRS}  # Use absolute path for LibTorch headers
#         #  ${LIBTORCH_API_INCLUDE_DIRS}  # Use absolute path for LibTorch API headers
# )

# # Set compile features
# target_compile_features(${MAIN_PROJECT_NAME}_${PROJECT_NAME} PUBLIC cxx_std_17)




# # Link libraries
# target_link_libraries(
#   ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
#   PUBLIC Eigen3::Eigen ${MAIN_PROJECT_NAME}::core ${MAIN_PROJECT_NAME}::math ${TORCH_LIBRARIES}
#   # ${LIBTORCH_LIBRARIES}
#   )

# # Set properties
# add_library(${MAIN_PROJECT_NAME}::${PROJECT_NAME} ALIAS
#             ${MAIN_PROJECT_NAME}_${PROJECT_NAME})
# set_target_properties(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
#                       PROPERTIES EXPORT_NAME ${PROJECT_NAME})


