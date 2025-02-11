/**
 * @file unit_tests.cpp
 * @brief The unit tests main file
 * @author sven.kalmbach
 * @date 2023-12-30
 */
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  if (!rclcpp::ok()) {
    throw std::runtime_error("Expected ROS to be ok");
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
