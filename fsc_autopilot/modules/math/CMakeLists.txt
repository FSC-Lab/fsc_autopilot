cmake_minimum_required(VERSION 3.20.0)
project(math)

# TODO(Hs293Go): Support FetchContent
find_package(Eigen3 REQUIRED)

add_library(${MAIN_PROJECT_NAME}_${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${MAIN_PROJECT_NAME}_${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
target_compile_features(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                        INTERFACE cxx_std_17)
target_link_libraries(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                      INTERFACE Eigen3::Eigen ${MAIN_PROJECT_NAME}::utils)

add_library(${MAIN_PROJECT_NAME}::${PROJECT_NAME} ALIAS
            ${MAIN_PROJECT_NAME}_${PROJECT_NAME})
set_target_properties(${MAIN_PROJECT_NAME}_${PROJECT_NAME}
                      PROPERTIES EXPORT_NAME ${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)
