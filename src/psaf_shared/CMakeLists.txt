cmake_minimum_required(VERSION 3.8)
project(psaf_shared)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  # c++17 required for std::filesystem
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/image_saver.cpp
  src/image_utils.cpp)

ament_target_dependencies(${PROJECT_NAME} geometry_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(unit_tests
    test/unit_tests_image_utils.cpp
    test/unit_tests_image_saver.cpp
    test/unit_tests_main.cpp
    src/image_saver.cpp
    src/image_utils.cpp)

    target_include_directories(unit_tests PRIVATE "include")
    set_tests_properties(unit_tests PROPERTIES ENVIRONMENT "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
    ament_target_dependencies(unit_tests OpenCV geometry_msgs)
    install(TARGETS
            unit_tests
            DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
