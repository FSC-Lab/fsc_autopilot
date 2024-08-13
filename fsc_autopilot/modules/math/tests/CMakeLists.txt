project(test_math)

find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME} src/test_rotation.cpp)
target_link_libraries(${PROJECT_NAME} GTest::gtest GTest::gtest_main
                      GTest::gmock fsc_autopilot::math)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})
