/**
 * @file unit_tests.cpp
 * @brief The unit tests for the watchdog node.
 * @author sven.kalmbach
 * @date 2024-03-01
 */

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "psaf_watchdog/watchdog_node.hpp"

/**
 * @brief Class used to test protected functions of WatchdogNode
 */
class WatchdogUnitTest : public ::testing::Test, public WatchdogNode
{
  void SetUp() override
  {
    auto cfg = WatchdogConfig();
    cfg.debugOutput = true;
    applyConfig(cfg);
  }
};

// test state update is applied to node
TEST_F(WatchdogUnitTest, TestStateUpdate) {
  // set inial state to known
  state = 0;
  const auto initialState = state;
  const auto newState = initialState + 1;

  const auto pubState = std::make_shared<std_msgs::msg::Int64>();
  pubState.get()->data = newState;

  // update node state and check is as expected
  updateState(pubState);
  EXPECT_EQ(state, newState) << "Expect state to change to new value";
  EXPECT_NE(state, initialState) << "Expect state to not be initial value";
}

// test unused callbacks dont cause publish or config change
TEST_F(WatchdogUnitTest, TestUnusedCallbacks) {
  // node to listen to errors
  auto errorNode = std::make_shared<rclcpp::Node>("test_watchdog_node");
  auto errorSub = errorNode->create_subscription<libpsaf_msgs::msg::Error>(
    WATCHDOG_ERROR_TOPIC, rclcpp::QoS(10), [&](
      libpsaf_msgs::msg::Error::SharedPtr error) -> void {
      (void)error;
      FAIL() << "No error callback expected during TestUnusedCallbacks";
    });

  // the valid config
  const auto cfg = getConfig();

  auto compareCfg = [&]() -> void {
      const auto newCfg = getConfig();
      EXPECT_EQ(newCfg.debugOutput, cfg.debugOutput);
      EXPECT_NEAR(
        newCfg.sensorMaxThreshold, cfg.sensorMaxThreshold,
        1e-6) << "Expect config value to remain unchanged.";
      EXPECT_NEAR(
        newCfg.sensorMinThreshold, cfg.sensorMinThreshold,
        1e-6) << "Expect config value to remain unchanged.";
      EXPECT_NEAR(
        newCfg.stopDistance, cfg.stopDistance,
        1e-6) << "Expect config value to remain unchanged.";
      EXPECT_NEAR(
        newCfg.updateFrequency, cfg.updateFrequency,
        1e-6) << "Expect config value to remain unchanged.";
    };

  // process image
  cv::Mat img(cv::Size(640, 480), CV_8UC3, cv::Scalar(255, 255, 255));
  processImage(img, 0);
  rclcpp::spin_some(errorNode);
  compareCfg();

  // update speed
  const auto pubSpeed = std::make_shared<std_msgs::msg::Int16>();
  pubSpeed.get()->data = 100;
  updateSpeed(pubSpeed);
  rclcpp::spin_some(errorNode);
  compareCfg();

  // update steering
  const auto pubSteering = std::make_shared<std_msgs::msg::Int16>();
  pubSteering.get()->data = 4;
  updateSteering(pubSteering);
  rclcpp::spin_some(errorNode);
  compareCfg();

  // update
  update();
  rclcpp::spin_some(errorNode);
  compareCfg();
}

// test simple stop: in config val and stops
TEST_F(WatchdogUnitTest, TestSimpleStop) {
  // set node to driving state
  state = 10;

  // apply config for this test
  auto cfg = getConfig();
  cfg.sensorMaxThreshold = 10;
  cfg.sensorMinThreshold = 1;
  cfg.stopDistance = 5;
  applyConfig(cfg);

  // generate certain stop value
  const double stopValue = cfg.stopDistance / 2;

  // create subscriber for error
  bool gotError = false;
  auto errorNode = std::make_shared<rclcpp::Node>("test_watchdog_node");
  auto errorSub = errorNode->create_subscription<libpsaf_msgs::msg::Error>(
    WATCHDOG_ERROR_TOPIC, rclcpp::QoS(10), [&](
      libpsaf_msgs::msg::Error::SharedPtr error) -> void {
      (void)error;
      gotError = true;
    });

  // create range msg without min and max range, should still work
  const auto rangeMsg = std::make_shared<sensor_msgs::msg::Range>();
  rangeMsg.get()->range = stopValue;
  rangeMsg.get()->min_range = 0;
  rangeMsg.get()->max_range = 0;

  // call callack and spin error node for result
  updateSensorValue(rangeMsg, 0);
  rclcpp::spin_some(errorNode);
  EXPECT_TRUE(gotError) << "Expect an error to be recieved";

  // test for message with range params
  gotError = false;
  rangeMsg.get()->min_range = 1;
  rangeMsg.get()->max_range = 10;

  // call callack and spin error node for result
  updateSensorValue(rangeMsg, 0);
  rclcpp::spin_some(errorNode);
  EXPECT_TRUE(gotError) << "Expect an error to be recieved";
}

// test dont process outside of range
// dont stop over range and dont stop out of range
TEST_F(WatchdogUnitTest, TestOutOfRange) {
  // set node to driving state
  state = 10;

  // apply config for this test
  auto cfg = getConfig();
  cfg.sensorMaxThreshold = 10;
  cfg.sensorMinThreshold = 1;
  cfg.stopDistance = 5;
  applyConfig(cfg);

  // create subscriber for error
  auto errorNode = std::make_shared<rclcpp::Node>("test_watchdog_node");
  auto errorSub = errorNode->create_subscription<libpsaf_msgs::msg::Error>(
    WATCHDOG_ERROR_TOPIC, rclcpp::QoS(10), [&](
      libpsaf_msgs::msg::Error::SharedPtr error) -> void {
      (void)error;
      FAIL() << "Dont expect error publish in this test";
    });

  // create range msg without min and max range, and stop value over range
  const auto rangeMsg = std::make_shared<sensor_msgs::msg::Range>();
  rangeMsg.get()->range = 12;
  rangeMsg.get()->min_range = 0;
  rangeMsg.get()->max_range = 0;
  updateSensorValue(rangeMsg, 0);
  rclcpp::spin_some(errorNode);

  // dont expect an error for stop value under range
  rangeMsg.get()->range = 0;
  updateSensorValue(rangeMsg, 0);
  rclcpp::spin_some(errorNode);

  // dont expect an error for stop value in range but over thresh
  rangeMsg.get()->range = 6;
  updateSensorValue(rangeMsg, 0);
  rclcpp::spin_some(errorNode);

  // dont expect error for invalid sensor
  rangeMsg.get()->range = 3;
  updateSensorValue(rangeMsg, 1);
  rclcpp::spin_some(errorNode);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (!rclcpp::ok()) {
    throw std::runtime_error("Expected ROS to be ok");
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
