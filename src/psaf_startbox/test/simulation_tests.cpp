/**
 * @file simulation_tests.cpp
 * @brief The simulation tests for the startbox node
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <sstream>

#include "gtest/gtest.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"
#include "sensor_msgs/msg/range.h"
#include "rmw/rmw.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "include/test_util.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "psaf_startbox/startbox_node.hpp"

std::vector<sensor_msgs::msg::Image> img_msgs_box_opens;
std::vector<sensor_msgs::msg::Range> range_msgs_box_opens;

/**
 * @class StartboxSimulationTests
 * @brief The simulation tests for the startbox node
 * @details This testsuite tests the startbox node with simulated data
 */
class StartBoxSimulationTests : public ::testing::Test
{
public:
  /**
   * @brief Setup the test suite. This method is only called once for the whole test suite.
   * @details in this methid the bags are being loaded, de-serialized and stored in the
   * corresponding vectors
   */
  static void SetUpTestCase()
  {
    std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
    std::string bag_images_box_opens, bag_range_box_opens;
    bag_images_box_opens = base_path + "/bags/qr_code";
    bag_range_box_opens = base_path + "/bags/startbox_open_us";

    deserializeRosImageBag(bag_images_box_opens, img_msgs_box_opens);
    deserializeRosRangeBag(bag_range_box_opens, range_msgs_box_opens);
  }
  /**
   * @brief Setup for the tests cases. Called before each test case.
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<StartBoxNode>();
    msgs_opens_image = img_msgs_box_opens;
    msgs_opens_range = range_msgs_box_opens;
  }
  /**
   * @brief Teardown for the tests cases. Called after each test case.
   */
  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<StartBoxNode> node;
  std::vector<sensor_msgs::msg::Image> msgs_opens_image;
  std::vector<sensor_msgs::msg::Range> msgs_opens_range;

private:
  /**
  * Read a ros bag, deserialize it and return a vector of sensor_msgs::msg::Image.
  * Code adapted from:
  * https://github.com/tiiuae/mission-data-recorder/blob/36e419dfb2d474d547a61ab4bc9a2e8e4f648983/internal/read_rosbag.cpp
  * @param bag_path Path to the baf
  * @param messages the vector of sensor_msgs::msg::Image
  */
  static void deserializeRosImageBag(
    std::string bag_path,
    std::vector<sensor_msgs::msg::Image> & messages)
  {
    rosbag2_storage::StorageOptions storage_options;
    rosbag2_cpp::readers::SequentialReader reader;

    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    // Create Factory
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    auto deserializer = factory.load_deserializer("cdr");

    auto type_support_library = rosbag2_cpp::get_typesupport_library(
      "sensor_msgs/msg/Image",
      "rosidl_typesupport_cpp");

    auto type_support_handle = rosbag2_cpp::get_typesupport_handle(
      "sensor_msgs/msg/Image",
      "rosidl_typesupport_cpp",
      type_support_library);

    while (reader.has_next()) {
      sensor_msgs::msg::Image msg;
      auto introspect = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
      introspect->time_stamp = 0;
      introspect->allocator = rcutils_get_default_allocator();
      introspect->message = &msg;

      auto serialized_msg = reader.read_next();

      deserializer->deserialize(serialized_msg, type_support_handle, introspect);

      messages.push_back(msg);
    }
  }

  /**
* Read a ros bag, deserialize it and return a vector of sensor_msgs::msg::Range.
* Code adapted from:
* https://github.com/tiiuae/mission-data-recorder/blob/36e419dfb2d474d547a61ab4bc9a2e8e4f648983/internal/read_rosbag.cpp
* @param bag_path Path to the baf
* @param messages the vector of sensor_msgs::msg::Range
*/
  static void deserializeRosRangeBag(
    std::string bag_path,
    std::vector<sensor_msgs::msg::Range> & messages)
  {
    rosbag2_storage::StorageOptions storage_options;
    rosbag2_cpp::readers::SequentialReader reader;

    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    // Create Factory
    rosbag2_cpp::SerializationFormatConverterFactory factory;
    auto deserializer = factory.load_deserializer("cdr");

    auto type_support_library = rosbag2_cpp::get_typesupport_library(
      "sensor_msgs/msg/Range",
      "rosidl_typesupport_cpp");

    auto type_support_handle = rosbag2_cpp::get_typesupport_handle(
      "sensor_msgs/msg/Range",
      "rosidl_typesupport_cpp",
      type_support_library);

    while (reader.has_next()) {
      sensor_msgs::msg::Range msg;
      auto introspect = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
      introspect->time_stamp = 0;
      introspect->allocator = rcutils_get_default_allocator();
      introspect->message = &msg;

      auto serialized_msg = reader.read_next();

      deserializer->deserialize(serialized_msg, type_support_handle, introspect);

      messages.push_back(msg);
    }
  }
};

/**
 * Test if the TestSuite can be created
 */
TEST_F(StartBoxSimulationTests, TestCanInitTestSuite)
{
  ASSERT_TRUE(true);
}

/**
 * Test if the read messages match the expected messages count
 */
TEST_F(StartBoxSimulationTests, TestImageBagCount)
{
  size_t image_bag_count = 74;
  ASSERT_EQ(image_bag_count, msgs_opens_image.size());
}

/**
 * Test if the read messages match the expected messages count
 */
TEST_F(StartBoxSimulationTests, TestRangeBagCount)
{
  size_t range_bag_count = 251;
  ASSERT_EQ(range_bag_count, msgs_opens_range.size());
}

/**
 * Test if the opening of the box is detected correctly
 */
TEST_F(StartBoxSimulationTests, TestCanDetectOpeningBoxWithImage)
{
  auto dummy = std::make_shared<rclcpp::Node>("dummy");
  auto publisher = dummy->create_publisher<sensor_msgs::msg::Image>("/color/image_raw", 1);
  int count = 0;
  for (auto msg : msgs_opens_image) {
    publisher->publish(msg);
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(30));
    if (!node->is_open_) {
      count++;
    }
  }
  ASSERT_EQ(46, count);
  ASSERT_TRUE(node->is_open_);
}

/**
 * Test if the opening of the box is detected correctly
 */
TEST_F(StartBoxSimulationTests, TestCanDetectOpenBoxWithUS)
{
  auto dummy = std::make_shared<rclcpp::Node>("dummy");
  auto publisher =
    dummy->create_publisher<sensor_msgs::msg::Range>("/uc_bridge/us_front_center", 1);

  for (auto msg : msgs_opens_range) {
    publisher->publish(msg);
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(30));
  }
  ASSERT_TRUE(node->is_open_);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
