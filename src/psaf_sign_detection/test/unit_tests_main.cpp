/**
 * @file unit_tests_main.cpp
 * @brief The unit tests main file
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // start ros for node unit tests
  rclcpp::init(0, nullptr);
  if (!rclcpp::ok()) {
    throw std::runtime_error("Expected ROS to be ok");
  }

  // start tests
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
