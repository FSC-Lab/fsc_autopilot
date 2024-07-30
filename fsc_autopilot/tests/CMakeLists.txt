project(fsc_autopilot_tests)

include(FetchContent)
FetchContent_Declare(
  GTest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.0
  FIND_PACKAGE_ARGS NAMES GTest 1.15.0)

FetchContent_MakeAvailable(GTest)

add_executable(${PROJECT_NAME} src/test_angles.cpp src/test_rotation.cpp
                               src/test_polynomial.cpp src/test_isclose.cpp)
target_link_libraries(${PROJECT_NAME} GTest::gtest GTest::gtest_main
                      GTest::gmock fsc_autopilot::math)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
