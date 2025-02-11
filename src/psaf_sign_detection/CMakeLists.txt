cmake_minimum_required(VERSION 3.5)
project(psaf_sign_detection)

set(NODE_NAME "sign_detection")

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
        src/sign_detector.cpp
        src/sign_id.cpp
        src/helpers.cpp)

target_include_directories(${NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${NODE_NAME} zbar ${OpenCV_LIBS})

ament_target_dependencies(${NODE_NAME} rclcpp std_msgs libpsaf OpenCV psaf_configuration psaf_shared)

install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(unit_tests
        src/${NODE_NAME}_node.cpp
        src/sign_detector.cpp
        src/sign_id.cpp
        src/helpers.cpp
        test/unit_test_helpers.cpp
        test/unit_test_sign_detection_node.cpp
        test/unit_test_sign_detector.cpp
        test/unit_test_sign_id.cpp
        test/unit_tests_main.cpp)

  target_include_directories(unit_tests PRIVATE "include")
  target_compile_definitions(unit_tests PRIVATE "BUILD_UNIT_TEST")
  set_tests_properties(unit_tests PROPERTIES ENVIRONMENT "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
  ament_target_dependencies(unit_tests libpsaf OpenCV psaf_configuration psaf_shared)

  install(TARGETS
        unit_tests
        DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
