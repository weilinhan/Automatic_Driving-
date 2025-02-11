/**
 * @file unit_tests_lane_kalman_filter.cpp
 * @brief The unit tests for the lane kalman filter
 * @author sven.kalmbach
 * @date 2023-12-30
 */
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "psaf_trajectory/lane_kalman_filter.hpp"

/**
 * @brief This class will setup the LaneKalmanFilter config for each test
 */
class LaneKalmanFilterUnitTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    if (filter == nullptr) {
      // NOTE: make sure to update test values if you change params
      // these are very close to the one on the car
      filter = new LaneKalmanFilter(0.1, 40.0, {50.0, 50.0}, {10.0, 0.0, 0.0, 0.0});
    } else {
      throw std::runtime_error("Invalid DUT state (setup)");
    }
  }

  void TearDown() override
  {
    if (filter != nullptr) {
      delete filter;
      filter = nullptr;
    } else {
      throw std::runtime_error("Invalid DUT state (teardown)");
    }
  }
  LaneKalmanFilter * filter = nullptr;
};

/**
 * @brief Test that the kalman filter behaves as expected with specified params
 */
TEST_F(LaneKalmanFilterUnitTest, TestSmoothLane) {
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = 10; p2.x = 20; p3.x = 100, p4.x = 40;
  p1.y = 0; p2.y = 0; p3.y = 0, p4.y = 0;
  p1.z = 0; p2.z = 0; p3.z = 0, p4.z = 0;
  std::vector<geometry_msgs::msg::Point> source = {p1, p2, p3, p4};

  // copy input
  auto input = source;
  ASSERT_GT(input.size(), 0U) << "Expect test input to be non zero";
  ASSERT_TRUE(filter->smoothLane(input)) << "Expect outlier to be detected";
  ASSERT_EQ(source.size(), input.size()) << "Expect number of points to remain the same";
  EXPECT_NEAR(input.at(2).x, 30, 10) << "Expect outlier to be fixed somewhat";

  // fix outlier manually
  p3.x = 30;
  source.at(2) = p3;
  input = source;
  ASSERT_GT(input.size(), 0U) << "Expect test input to be non zero";
  ASSERT_FALSE(filter->smoothLane(input)) << "Expect no outlier to be detected";
  ASSERT_EQ(source.size(), input.size()) << "Expect number of points to remain the same";
}

TEST_F(LaneKalmanFilterUnitTest, TestEmpty) {
  std::vector<geometry_msgs::msg::Point> input;
  EXPECT_TRUE(filter->smoothLane(input)) << "Expect false retval for empty lane";
}
