/**
 * @file integration_tests.cpp
 * @brief The integration tests for the startbox node
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "gtest/gtest.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.h"
#include "std_msgs/msg/int64.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "psaf_startbox/startbox_node.hpp"
#include "opencv4/opencv2/opencv.hpp"

/**
 * @class StartboxIntegrationTests
 * @brief The integration tests for the startbox node
 * @details This testsuite contains the test cases for the publisher and subscriber of the startbox node
 */
class StartboxIntegrationTest : public ::testing::Test
{
public:
  /**
   * Setup for the tests case. Called before each test case.
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<StartBoxNode>();
  }

  /**
   * Teardown for the tests case. Called after each test case.
   */
  void TearDown() override
  {
    rclcpp::shutdown();
  }

  // The test object
  std::shared_ptr<StartBoxNode> node;
};

/**
 * Test if the startbox node can be created
 */
TEST(StartboxStartUpTest, TestNodeCanBeCreated)
{
  // Init ros
  rclcpp::init(0, nullptr);
  // Create a ros graph
  rclcpp::node_interfaces::NodeGraph * node_graph_;
  // Create the startbox node
  std::shared_ptr<StartBoxNode> node = std::make_shared<StartBoxNode>();
  // Initialize the node graph
  node_graph_ =
    dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node->get_node_graph_interface().get());
  // Check if the node graph is valid -> not a nullptr
  ASSERT_NE(nullptr, node_graph_);
  // Set the spin rate to 30ms
  rclcpp::WallRate rate(30.0);
  // Create a vector to store the node names
  std::vector<std::string> node_names;
  // get current time, used for shutdown
  auto now = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    node_names = node_graph_->get_node_names();
    rate.sleep();
    // after 3 seconds, stop the node
    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now() -
        now).count() > 3)
    {
      break;
    }
  }
  rclcpp::shutdown();

// Check if /startbox is in the node names
  ASSERT_NE(
    std::find(node_names.begin(), node_names.end(), "/startbox"),
    node_names.end()) << "Expected the /startbox in node names";
}

/**
 * Test if the testsuite can be set up
 */
TEST_F(StartboxIntegrationTest, TestCanInitTestSuite)
{
  ASSERT_TRUE(true);
}

/**
 * Test if the startbox node has the right amount of topics
 */
TEST_F(StartboxIntegrationTest, TestTopicCount)
{
  std::map<std::string, std::vector<std::string>> topic_infos = node->get_topic_names_and_types();
  int count = static_cast<int>(topic_infos.size());
  ASSERT_EQ(count, 6 + NBR_OF_US_SENSORS) <<
    "Expected topic count does not match actual topic count";
}

/**
 * Tests if the Image Subscriber works
 */
TEST_F(StartboxIntegrationTest, TestCanReceiveQRCodeImageMessageAndDecode)
{
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto publisher = dummy->create_publisher<sensor_msgs::msg::Image>("/color/image_raw", 1);
  cv::Mat image_qr_code, image_no_qr_code;
  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  std::string qr_code_path = base_path + "/images/qr_code.png";
  std::string no_qr_code_path = base_path + "/images/no_qr_code.png";
  image_qr_code = cv::imread(qr_code_path, cv::IMREAD_COLOR);
  image_no_qr_code = cv::imread(no_qr_code_path, cv::IMREAD_COLOR);

  int msg_val{-1};
  auto subscriber = dummy->create_subscription<libpsaf_msgs::msg::StatusInfo>(
    "/status/status_info", 10, [&msg_val](libpsaf_msgs::msg::StatusInfo::SharedPtr msg) {
      msg_val = msg->type;
    });

  sensor_msgs::msg::Image msg;
  std_msgs::msg::Header header;
  header.frame_id = "qr_code";
  header.stamp = rclcpp::Time(0);
  std::string encoding = "bgr8";
  cv_bridge::CvImage cv_image(header, encoding, image_qr_code);
  cv_bridge::CvImage cv_image_no_qr_code(header, encoding, image_no_qr_code);
  rclcpp::WallRate rate(30.0);
  int counter{0};
  while (rclcpp::ok()) {
    if (counter < 300) {
      msg = *(cv_image.toImageMsg());
    } else {
      msg = *(cv_image_no_qr_code.toImageMsg());
    }
    publisher->publish(msg);
    rclcpp::spin_some(dummy);
    node->update();
    rclcpp::spin_some(node);
    rate.sleep();
    counter++;
    if (msg_val == libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN) {
      break;
    }
  }
  ASSERT_TRUE(node->is_open_);
}

/**
 * Test if the US Sensor Subscriber works
 */
TEST_F(StartboxIntegrationTest, TestCanReceiveUSMessage)
{
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto publisher = dummy->create_publisher<sensor_msgs::msg::Range>("uc_bridge/us_front_center", 1);
  int msg_val{-1};
  auto subscriber = dummy->create_subscription<libpsaf_msgs::msg::StatusInfo>(
    "/status/status_info", 10, [&msg_val](libpsaf_msgs::msg::StatusInfo::SharedPtr msg) {
      msg_val = msg->type;
    });
  sensor_msgs::msg::Range msg;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = 0.1;
  msg.min_range = 0.0;
  msg.max_range = 10.0;
  msg.range = 0.35;
  rclcpp::WallRate rate(30.0);
  while (rclcpp::ok()) {
    publisher->publish(msg);
    rclcpp::spin_some(dummy);
    node->update();
    rclcpp::spin_some(node);
    rate.sleep();
    if (msg_val == libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN) {
      break;
    }
  }
  ASSERT_TRUE(node->is_open_);
}

/**
 * Test if the node shuts down correctly after a different state is entered
 */
TEST_F(StartboxIntegrationTest, TestCanReceiveStateAndShutsDown)
{
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto publisher = dummy->create_publisher<std_msgs::msg::Int64>(
    "state_machine/state", 1);

  std_msgs::msg::Int64 msg;
  msg.data = 12;
  publisher->publish(msg);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);
  node->update();
  ASSERT_TRUE(!rclcpp::ok());
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
