cmake_minimum_required(VERSION 3.24.0)
project(utils)

add_library(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_compile_features(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                        INTERFACE cxx_std_17)
# The utils module MUST NOT LINK ANYTHING target_link_libraries()

add_library(${MAIN_PROJECT_NAME}::${PROJECT_NAME} ALIAS
            ${MAIN_PROJECT_NAME}_${PROJECT_NAME})
set_target_properties(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                      PROPERTIES EXPORT_NAME ${PROJECT_NAME})
