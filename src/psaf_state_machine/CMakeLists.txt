cmake_minimum_required(VERSION 3.5)
project(psaf_state_machine)

set(NODE_NAME "psaf_state_machine")

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

set(psaf_basic_state_machine_SRCs
    src/basic_cup/basic_cup_state_machine.cpp
    src/basic_cup/states/start_box.cpp
    src/basic_cup/states/driving.cpp
    src/basic_cup/states/error.cpp
    src/basic_cup/states/overtake.cpp
    src/basic_cup/states/parking.cpp
    src/basic_cup/states/manual_driving.cpp
    src/basic_cup/states/shutdown.cpp)

set(basic_cup_state_machine_full_SRCs
    src/basic_cup/basic_cup_state_machine_node.cpp
    src/basic_cup/basic_cup_state_machine_main.cpp
    ${psaf_basic_state_machine_SRCs})

set(integration_test_SRCs
    src/basic_cup/basic_cup_state_machine.cpp
    src/basic_cup/basic_cup_state_machine_node.cpp
    ${psaf_basic_state_machine_SRCs})




add_executable(basic_cup_state_machine ${basic_cup_state_machine_full_SRCs})
target_include_directories(basic_cup_state_machine PRIVATE "include")
ament_target_dependencies(basic_cup_state_machine rclcpp std_msgs sensor_msgs libpsaf_msgs libpsaf psaf_configuration)


install(TARGETS
        basic_cup_state_machine
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

  # GTEST
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp ${psaf_basic_state_machine_SRCs})
  target_include_directories(test_state_machine  PRIVATE "include")
  ament_target_dependencies(test_state_machine libpsaf_msgs rclcpp psaf_configuration)

  ament_add_gtest(test_drive_with_park test/test_state_machine_discipline_one.cpp ${psaf_basic_state_machine_SRCs})
  target_include_directories(test_drive_with_park PRIVATE "include")
  ament_target_dependencies(test_drive_with_park libpsaf_msgs rclcpp psaf_configuration)

  ament_add_gtest(test_obstacle_evasion_course test/test_state_machine_discipline_two.cpp ${psaf_basic_state_machine_SRCs})
  target_include_directories(test_obstacle_evasion_course PRIVATE "include")
  ament_target_dependencies(test_obstacle_evasion_course libpsaf_msgs rclcpp psaf_configuration)

  ament_add_gtest(integration_test test/integration_test.cpp ${integration_test_SRCs})
  target_include_directories(integration_test PUBLIC "include")
  ament_target_dependencies(integration_test rclcpp libpsaf_msgs libpsaf psaf_configuration)
  set_tests_properties(integration_test PROPERTIES TIMEOUT 600)


  install(TARGETS
          test_state_machine
          test_drive_with_park
          test_obstacle_evasion_course
          integration_test
          DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()
