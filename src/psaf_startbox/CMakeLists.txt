cmake_minimum_required(VERSION 3.5)
project(psaf_startbox)

set(NODE_NAME "startbox")

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
find_package(sensor_msgs REQUIRED)
find_package(libpsaf REQUIRED)
find_package(libpsaf_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(cv_bridge 3.2.1 REQUIRED)

add_executable(${NODE_NAME}
        src/${NODE_NAME}.cpp
        src/${NODE_NAME}_node.cpp)

target_include_directories(${NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${NODE_NAME} zbar ${OpenCV_LIBS})

ament_target_dependencies(${NODE_NAME} rclcpp std_msgs libpsaf OpenCV psaf_configuration)

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

  ament_add_gtest(startbox_unit_tests test/unit_tests.cpp src/startbox_node.cpp)
  ament_add_gtest(startbox_integration_tests test/integration_tests.cpp src/startbox_node.cpp)
  ament_add_gtest(startbox_simulation_tests test/simulation_tests.cpp src/startbox_node.cpp)


  target_include_directories(startbox_unit_tests PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${OpenCV_INCLUDE_DIRS})

  target_include_directories(startbox_integration_tests PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${OpenCV_INCLUDE_DIRS})

  target_include_directories(startbox_simulation_tests PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${OpenCV_INCLUDE_DIRS})



  target_link_libraries(startbox_unit_tests zbar ${OpenCV_LIBS})
  target_link_libraries(startbox_integration_tests zbar ${OpenCV_LIBS})
  target_link_libraries(startbox_simulation_tests zbar ${OpenCV_LIBS})



  ament_target_dependencies(startbox_unit_tests OPENCV rclcpp libpsaf libpsaf_msgs psaf_configuration)

  ament_target_dependencies(startbox_integration_tests OPENCV rclcpp libpsaf libpsaf_msgs std_msgs sensor_msgs cv_bridge psaf_configuration)

  ament_target_dependencies(startbox_simulation_tests OPENCV rclcpp libpsaf rosbag2_cpp cv_bridge libpsaf_msgs psaf_configuration)


  set_tests_properties(startbox_unit_tests PROPERTIES
          ENVIRONMENT
          "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  set_tests_properties(startbox_integration_tests PROPERTIES
          ENVIRONMENT
          "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  set_tests_properties(startbox_simulation_tests PROPERTIES
          ENVIRONMENT
          "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  install(TARGETS
          startbox_unit_tests
          startbox_integration_tests
          startbox_simulation_tests
          DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
