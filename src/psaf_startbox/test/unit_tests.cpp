/**
 * @file unit_tests.cpp
 * @brief The unit tests for the startbox node
 * @author PSAF
 * @date 2022-06-01
 */
#include <string>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "psaf_startbox/startbox_node.hpp"
#include "opencv4/opencv2/opencv.hpp"

/**
 * @class StartboxUnitTests
 * @brief The unit tests for the startbox node
 * @details This testsuite contains the unit tests for the startbox node
 */
class StartboxUnitTests : public StartBoxNode, public ::testing::Test
{
public:
  /**
   * Setup for the tests case. Called before each test case.
   */
  void SetUp() override
  {
    base_path = std::string(std::getenv("TEST_DATA_DIR"));
  }
  std::string base_path;
};

/**
 * Test if the Unit Tests can be initialized
 */
TEST_F(StartboxUnitTests, TestCanSetupTestSuite)
{
  ASSERT_EQ(0, 0);
}

/**
 * Test if the QR code is detected correctly
 */
TEST_F(StartboxUnitTests, TestCanReadQRCode)
{
  cv::Mat image_qr_code, gray_image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  cv::cvtColor(image_qr_code, gray_image_qr_code, cv::COLOR_BGR2GRAY);
  readQR(gray_image_qr_code);
  ASSERT_EQ(last_read_qr_, "STOP");
}

/**
 * Test if no QR Code is detected if none is given
 */
TEST_F(StartboxUnitTests, TestDoesNotDetectQRCode)
{
  cv::Mat image_qr_code, gray_image_qr_code;
  std::string qr_code_path = base_path + "/images/no_qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  cv::cvtColor(image_qr_code, gray_image_qr_code, cv::COLOR_BGR2GRAY);
  readQR(gray_image_qr_code);
  ASSERT_EQ(last_read_qr_, "");
}

/**
 * Test if the QR Code Reader can read other QR Codes
 */
TEST_F(StartboxUnitTests, TestCanDetectQRCodeNotStop)
{
  cv::Mat image_qr_code, gray_image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code_test.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  cv::cvtColor(image_qr_code, gray_image_qr_code, cv::COLOR_BGR2GRAY);
  readQR(gray_image_qr_code);
  ASSERT_EQ(last_read_qr_, "TEST");
}

/**
 * Test if the flag is set correctly
 */
TEST_F(StartboxUnitTests, TestSetsReadFlagCorrectly)
{
  cv::Mat image_qr_code, gray_image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  cv::cvtColor(image_qr_code, gray_image_qr_code, cv::COLOR_BGR2GRAY);
  readQR(gray_image_qr_code);
  ASSERT_TRUE(detected_at_least_once_);
}

/**
 * After 10 successful reads, the gate can open and the flag should be set.
 */
TEST_F(StartboxUnitTests, TestCanSetOpenCorrectly)
{
  cv::Mat image_qr_code, image_no_qr_code, gray_image_qr_code, gray_image_no_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code.png";
  std::string no_qr_code_path = base_path + "/images/no_qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  image_no_qr_code = cv::imread(no_qr_code_path, cv::IMREAD_COLOR);
  cv::cvtColor(image_qr_code, gray_image_qr_code, cv::COLOR_BGR2GRAY);
  cv::cvtColor(image_no_qr_code, gray_image_no_qr_code, cv::COLOR_BGR2GRAY);
  readQR(gray_image_qr_code);
  no_qr_msg_counter_ = 10;
  readQR(gray_image_no_qr_code);
  ASSERT_TRUE(is_open_);
}

/**
 * Test that the image is only processed if the sensor is set to 0
 */
TEST_F(StartboxUnitTests, TestCanProcessSensorZero)
{
  cv::Mat image_qr_code;
  std::string qr_code_path = base_path + "/images/no_qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  processImage(image_qr_code, 0);
  ASSERT_EQ(last_read_qr_, "");
  ASSERT_FALSE(detected_at_least_once_);
}

/**
 * Test that there is no reaction for other sensor values than 0
 */
TEST_F(StartboxUnitTests, TestCanProcessSensorOne)
{
  cv::Mat image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code_test.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  processImage(image_qr_code, 1);
  ASSERT_EQ(last_read_qr_, "INIT");
  ASSERT_FALSE(detected_at_least_once_);
}

/**
 * Tests that an empty image won´t crash the program
 */
TEST_F(StartboxUnitTests, TestCanReactToEmptyImage)
{
  cv::Mat empty;
  processImage(empty, 0);
  ASSERT_EQ(last_read_qr_, "INIT");
  ASSERT_FALSE(detected_at_least_once_);
}

/**
 * Test that images with wrong width will get processed
 */
TEST_F(StartboxUnitTests, TestCanReactToWrongWidth)
{
  cv::Mat image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  // Double image width
  cv::resize(image_qr_code, image_qr_code, cv::Size(), 2, 1, cv::INTER_NEAREST);
  processImage(image_qr_code, 0);
  ASSERT_EQ(last_read_qr_, "STOP");
  ASSERT_TRUE(detected_at_least_once_);
}

/**
 * Test that images with wrong height will get processed
 */
TEST_F(StartboxUnitTests, TestCanReactToWrongHeight)
{
  cv::Mat image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  // Double image height
  cv::resize(image_qr_code, image_qr_code, cv::Size(), 1, 2, cv::INTER_NEAREST);
  processImage(image_qr_code, 0);
  ASSERT_EQ(last_read_qr_, "STOP");
  ASSERT_TRUE(detected_at_least_once_);
}

/**
 * Test that images with wrong width and height will get processed
 */
TEST_F(StartboxUnitTests, TestCanReactToWrongWidthAndHeight)
{
  cv::Mat image_qr_code;
  std::string qr_code_path = base_path + "/images/qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  // Double image width and height
  cv::resize(image_qr_code, image_qr_code, cv::Size(), 2, 2, cv::INTER_NEAREST);
  processImage(image_qr_code, 0);
  ASSERT_EQ(last_read_qr_, "STOP");
  ASSERT_TRUE(detected_at_least_once_);
}

/**
 * Test the correct reaction when a barcode is read instead of a QR code
 */
TEST_F(StartboxUnitTests, TestReadBarCodeButDoesNotReact)
{
  cv::Mat image_bar_code, gray_image_bar_code;
  std::string bar_code_path = base_path + "/images/barcode.png";
  image_bar_code = cv::imread(bar_code_path, cv::IMREAD_COLOR);
  cv::cvtColor(image_bar_code, gray_image_bar_code, cv::COLOR_BGR2GRAY);
  readQR(gray_image_bar_code);
  ASSERT_EQ(last_read_qr_, "INIT");
  ASSERT_FALSE(detected_at_least_once_);
}

/**
 * Test the receiving of a state
 */
TEST_F(StartboxUnitTests, TestCanAssignState)
{
  int state = 42;
  std_msgs::msg::Int64 msg;
  msg.data = state;
  updateState(std::make_shared<std_msgs::msg::Int64>(msg));
  ASSERT_EQ(current_state_, state);
}

/**
 * Call Update with open is true
 */
TEST_F(StartboxUnitTests, TestCanCallUpdate)
{
  is_open_ = true;
  ASSERT_NO_THROW(update());
}

/**
 * Call Update when open is false
 */
TEST_F(StartboxUnitTests, TestCanCallUpdateWithFalseIsOpen)
{
  is_open_ = false;
  ASSERT_NO_THROW(update());
}

/**
 * Test the US Sensor Subscriber
 */
TEST_F(StartboxUnitTests, TestDoesIgnoreIfSensorIsNotZero)
{
  int sensor = 1;
  sensor_msgs::msg::Range msg;
  msg.range = 10.0;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(last_received_distance_, 0.0);
}

/**
 * If the sensor is 0, the distance should be updated
 */
TEST_F(StartboxUnitTests, TestDoesReactToZeroSensor)
{
  int sensor = 0;
  sensor_msgs::msg::Range msg;
  msg.range = 100.0;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(last_received_distance_, 100.0);
}

/**
 * Zero distance should be ignored
 */
TEST_F(StartboxUnitTests, TestIgnoresZeroRange)
{
  int sensor = 0;
  last_received_distance_ = -1.0;
  sensor_msgs::msg::Range msg;
  msg.range = 0.0;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(last_received_distance_, -1.0);
}

/**
 * Test if the car ignores a negative range reading
 */
TEST_F(StartboxUnitTests, TestIgnoredNegativeRange)
{
  int sensor = 0;
  last_received_distance_ = 5.0;
  sensor_msgs::msg::Range msg;
  msg.range = -10.0;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(last_received_distance_, 5.0);
}

/**
 * Update the counter if range is greater than 30 cm
 */
TEST_F(StartboxUnitTests, TestIncreasesCounterIfRangeGreaterThirty)
{
  int sensor = 0;
  us_msg_counter_ = 0;
  sensor_msgs::msg::Range msg;
  msg.range = 0.31;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 1);
}

/**
 * Counter should not be increased if range is less than 30 cm
 */
TEST_F(StartboxUnitTests, TestDoesNotIncreaseCounterIfRangeLessThanThirty)
{
  int sensor = 0;
  us_msg_counter_ = 0;
  sensor_msgs::msg::Range msg;
  msg.range = 0.29;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 0);
}

/**
 * Counter should be increased if range is 30 cm
 */
TEST_F(StartboxUnitTests, TestDoesIncreaseCounterIfRangeIsThirty)
{
  int sensor = 0;
  us_msg_counter_ = 0;
  sensor_msgs::msg::Range msg;
  msg.range = 0.3;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 1);
}

/**
 * Test multiple valid measurements
 */
TEST_F(StartboxUnitTests, TestCanIncreaseMultipleTimes)
{
  int sensor = 0;
  us_msg_counter_ = 0;
  sensor_msgs::msg::Range msg;
  msg.range = 0.31;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 1);
  msg.range = 0.32;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 2);
  msg.range = 0.3;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 3);
}

/**
 * Test if the counter is reset if the distance is less than 30 cm
 */
TEST_F(StartboxUnitTests, TestCanResetCounterToZeroIfRangeWasLessThanThirty)
{
  int sensor = 0;
  us_msg_counter_ = 3;
  sensor_msgs::msg::Range msg;
  msg.range = 0.29;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 0);
}

/**
 * Test if the open flag is set after 11 valid measurements
 */
TEST_F(StartboxUnitTests, TestSetIsOpenAfterElevenMeasurementsOverThirty)
{
  int sensor = 0;
  us_msg_counter_ = 10;
  sensor_msgs::msg::Range msg;
  msg.range = 0.31;
  updateSensorValue(std::make_shared<sensor_msgs::msg::Range>(msg), sensor);
  ASSERT_EQ(us_msg_counter_, 11);
  ASSERT_TRUE(is_open_);
}

// Test apply config detects edge case
TEST_F(StartboxUnitTests, TestApplyConfig) {
  StartBoxConfig cfg;
  cfg.ignoreDistance = true;
  cfg.ignoreQr = true;

  bool gotException = false;
  try {
    applyConfig(cfg);
  } catch (...) {
    gotException = true;
  }
  EXPECT_TRUE(gotException) <<
    "Expect an exception when both modes of startbox detection are disabled";

  cfg.ignoreDistance = false;
  gotException = false;
  try {
    applyConfig(cfg);
  } catch (...) {
    gotException = true;
  }
  EXPECT_FALSE(gotException) <<
    "Dont expect exception when one mode of startbox detection is enabled";
}

/**
 * Node should shutdown if the state changed from STARTBOX to save ressources
 */
TEST_F(StartboxUnitTests, TestShutdownCanBeCalled)
{
  current_state_ = 42;
  update();
  ASSERT_FALSE(rclcpp::ok());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
