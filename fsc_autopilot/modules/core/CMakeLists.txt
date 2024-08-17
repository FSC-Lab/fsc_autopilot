cmake_minimum_required(VERSION 3.20.0)
project(core)

add_library(${MAIN_PROJECT_NAME}_${PROJECT_NAME} src/logger_base.cpp
                                                 src/vehicle_model.cpp)
target_include_directories(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_compile_features(${MAIN_PROJECT_NAME}_${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME} PUBLIC ${MAIN_PROJECT_NAME}::utils
                                              ${MAIN_PROJECT_NAME}::math)

add_library(${MAIN_PROJECT_NAME}::${PROJECT_NAME} ALIAS
            ${MAIN_PROJECT_NAME}_${PROJECT_NAME})
set_target_properties(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                      PROPERTIES EXPORT_NAME ${PROJECT_NAME})
