cmake_minimum_required(VERSION 3.5)
project(psaf_trajectory)

set(NODE_NAME "trajectory")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(libpsaf REQUIRED)
find_package(libpsaf_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(psaf_shared REQUIRED)

add_executable(${NODE_NAME}
        src/${NODE_NAME}.cpp
        src/${NODE_NAME}_node.cpp
        src/lane_kalman_filter.cpp)

target_include_directories(${NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(${NODE_NAME} rclcpp std_msgs libpsaf psaf_configuration psaf_shared)

install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(unit_tests
        test/unit_tests.cpp
        test/unit_tests_lane_kalman_filter.cpp
        test/unit_tests_trajectory_node.cpp
        src/${NODE_NAME}_node.cpp
        src/lane_kalman_filter.cpp)

  target_include_directories(unit_tests PRIVATE "include")
  target_compile_definitions(unit_tests PRIVATE "BUILD_UNIT_TEST")
  ament_target_dependencies(unit_tests rclcpp std_msgs libpsaf psaf_configuration psaf_shared)
  set_tests_properties(unit_tests PROPERTIES ENVIRONMENT "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  install(TARGETS
          unit_tests
          DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
