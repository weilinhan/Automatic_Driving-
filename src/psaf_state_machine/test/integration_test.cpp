/**
 * @file psaf_state_machine/test/integration_test.cpp
 * @brief the integration tests for the state machine.
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <ctime>

#include "include/integration_test.hpp"
#include "include/random_tests.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include "libpsaf_msgs/msg/sign.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int64.hpp"
#include "include/test_util.hpp"

#include "gtest/gtest.h"

using std::placeholders::_1;


/**
 * @class DummyPubSub
 * @brief A simple Dummy class for sending and receiving messages.
 * Acts as a dummy node to test the correct response of the state_machine
 * Contains a button and a status info publisher
 */
class DummyPubSub : public rclcpp::Node
{
public:
  DummyPubSub()
  : Node("dummy_publisher")
  {
    status_publisher_ = this->create_publisher<libpsaf_msgs::msg::StatusInfo>(
      "/status/status_info",
      rclcpp::QoS(rclcpp::KeepLast{10}));
    button_publisher_ =
      this->create_publisher<std_msgs::msg::Int8>(
      "/uc_bridge/button",
      rclcpp::QoS(rclcpp::KeepLast{10}));

    error_publisher_ =
      this->create_publisher<libpsaf_msgs::msg::Error>(
      "/watchdog/error_message",
      rclcpp::QoS(rclcpp::KeepLast{10}));

    state_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
      STATE_TOPIC, rclcpp::QoS(rclcpp::KeepLast{10}),
      std::bind(&DummyPubSub::state_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Started DummyPubSub");
  }

  /**
   * Published the given status info to test the state_machine
   * @param type
   */
  void publish_status(int type)
  {
    libpsaf_msgs::msg::StatusInfo msg;
    msg.type = type;
    status_publisher_->publish(msg);
    has_received = false;
  }

  /**
   * Publishes the given button event
   * @param button the button type, must be 0 or 1
   */
  void publish_button(int button)
  {
    if (button != 0 && button != 1) {
      RCLCPP_ERROR(this->get_logger(), "Button value must be 0 or 1");
      return;
    }
    std_msgs::msg::Int8 msg;
    msg.data = button;
    button_publisher_->publish(msg);
  }

  /**
   * Publishes the given error message
   * @param[in] type the error type
   */
  void publish_error(int type)
  {
    libpsaf_msgs::msg::Error error;
    error.type = type;
    error_publisher_->publish(error);
    has_received = false;
  }

  /**
   * Returns the current state of the state_machine
   * Received on the callback
   * @return The current state of the state machine
   */
  int get_state()
  {
    has_received = false;
    return state;
  }

  /**
   * True if a message was received on the state_machine
   * @return
   */
  bool msg_received()
  {
    return has_received;
  }

private:
  /**
   * Callback for state changes issued by the state_machine
   * @param msg
   */
  void state_callback(std_msgs::msg::Int64::SharedPtr msg)
  {
    has_received = true;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    state = msg->data;
  }

  int state = -1;
  bool has_received = false;
  rclcpp::Publisher<libpsaf_msgs::msg::StatusInfo>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr button_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr state_subscriber_;
  rclcpp::Publisher<libpsaf_msgs::msg::Error>::SharedPtr error_publisher_;
};

void wait_for_response(
  std::shared_ptr<BasicCupStateMachineNode> node,
  std::shared_ptr<DummyPubSub> dummy);


/**
 * @class IntegrationTest
 * @brief The integration testsuite for the state_machine
 * @details Integration test class for the state_machine. These tests check, if the state_machine reacts to
 * external events according to the specification
 */
class IntegrationTest : public ::testing::Test
{
public:
  /**
   * Setup the testcases by initializing ros
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<BasicCupStateMachineNode>();
    rclcpp::WallRate rate(std::chrono::milliseconds(30));
  }

  /**
   * Teardown after each testcase
   */
  void TearDown() override
  {
    rclcpp::shutdown();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::shared_ptr<BasicCupStateMachineNode> node;
  std::shared_ptr<DummyPubSub> dummy_node;
};

/**
 * Test if the state_machine node has the correct name
 */
TEST_F(IntegrationTest, TestCheckNodeName)
{
  rclcpp::spin_some(node);
  ASSERT_STREQ("state_machine", node->get_name()) << "Expected node name to be state_machine";
}

/**
 * Test if the state_machine has the right amount of topics
 */
TEST_F(IntegrationTest, TestTopicCount)
{
  rclcpp::spin_some(node);
  // Adapted from https://answers.ros.org/question/370302/ros2-how-to-get-all-topic-names/
  std::map<std::string, std::vector<std::string>> topic_infos = node->get_topic_names_and_types();

  int count = 0;
  for (const auto & topic_it : topic_infos) {
    std::string topic_name = topic_it.first;
    if (topic_name.compare("/parameter_events") == 0 || topic_name.compare("/rosout") == 0) {
      continue;
    }
    count++;
  }
  ASSERT_EQ(count, 5) << "Expected a total topic count of 5, including rosout and parameter_events";
}

/**
 * Test if the car is reacting to a status info, if no button was pressed.
 * There should be NO reaction --> stays in state 0
 */
TEST_F(IntegrationTest, TestStaysInStartBoxState)
{
  std::vector<int> values;
  for (int i = 0; i < 26; i++) {
    if (i == 23 || i == 24) {
      continue;
    }
    values.push_back(i);
  }
  auto data = generate_messages(values);

  auto result = manage_run(node, data);

  for (auto msgre : result) {
    ASSERT_EQ(msgre.data, 0) << "Expected car to stay in state STARTBOX = 0";
  }
}

/**
 * Test if the Car leaves the startbox
 */
TEST_F(IntegrationTest, TestLeavesStartBoxState)
{
  publish_button_once(node, 0);
  std::vector<int> sequence{0};
  std::vector<libpsaf_msgs::msg::StatusInfo> values = generate_messages(sequence);

  auto result = manage_run(node, values);

  ASSERT_EQ(result.at(0).data, 10) << "Expected car to transition into state DR_NORMAL = 10";
}

/**
 * Test if the car can complete a normal full drive. This includes
 * the possible up and downhill in the MasterCup
 */
TEST_F(IntegrationTest, TestFullDisciplineOneParallelPark)
{
  publish_button_once(node, 0);
  auto data = generate_messages(full_run_park_parallel);
  auto result = manage_run(node, data);
  std::vector<int> expected{10, 11, 12, 10, 15, 16, 18, 19, 10};
  int i = 0;
  for (auto msg : result) {
    ASSERT_EQ(
      msg.data,
      expected.at(i)) <<
      "Expected car to perform normal drive with parking. Failed in iteration: " << i;
    i++;
  }
}

/**
 * Test the Perpendicular Parking Sequence
 */
TEST_F(IntegrationTest, TestFullDisciplineOnePerpendicularPark)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  auto data = generate_messages(full_run_park_perpendicular);
  auto result = manage_run(node, data);
  std::vector<int> expected{10, 15, 17, 18, 19, 10};
  int i = 0;
  for (auto msg : result) {
    ASSERT_EQ(
      msg.data,
      expected.at(i)) << "Expected car to follow normal drive. Failed in iteration: " << i;
    i++;
  }
}

/**
 * Test the abort if no parking spot was found
 */
TEST_F(IntegrationTest, TestNoParkingSpotFound)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  auto data = generate_messages(no_spot_found);
  auto result = manage_run(node, data);
  std::vector<int> expected{10, 15, 10};
  int i = 0;
  for (auto msg : result) {
    ASSERT_EQ(
      msg.data,
      expected.at(i)) << "Expected car to abort parking attempt. Failed in iteration: " << i;
    i++;
  }
}

/**
 * Test Failure of Parallel Parking
 */
TEST_F(IntegrationTest, TestParallelParkingFailed)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  auto data = generate_messages(parking_failed_parallel);
  auto result = manage_run(node, data);
  std::vector<int> expected{10, 15, 16, 19, 10};
  int i = 0;
  for (auto msg : result) {
    ASSERT_EQ(msg.data, expected.at(i)) << "Expected car to abort. Failed in iteration: " << i;
    i++;
  }
}

/**
 * Test Failure of Perpendicular Parking
 */
TEST_F(IntegrationTest, TestPerpendicularParkingFailed)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  auto data = generate_messages(parking_failed_perpendicular);
  auto result = manage_run(node, data);
  std::vector<int> expected{10, 15, 17, 19, 10};
  int i = 0;
  for (auto msg : result) {
    ASSERT_EQ(msg.data, expected.at(i)) << "Expected car to abort. Failed in iteration: " << i;
    i++;
  }
}

/**
 * Test if manual mode can be entered from the startbox
 */
TEST_F(IntegrationTest, TestTransitIntoManualModeFromStartBox)
{
  std::vector<int> values{23};
  auto data = generate_messages(values);
  auto result = manage_run(node, data);
  ASSERT_EQ(result.at(0).data, 2) << "Expected car to transition into MANUAL_MODE = 2";
}

/**
 * Test if manual mode is reachable from every state in discipline one
 */
TEST_F(IntegrationTest, TestTransitIntoManualModeDisciplineOne)
{
  std::vector<int> currently_selected;
  for (size_t i = 0; i < all_state_discipline_one.size(); i++) {
    // Setup
    currently_selected.clear();
    node = nullptr;
    node = std::make_shared<BasicCupStateMachineNode>();
    rclcpp::spin_some(node);
    publish_button_once(node, 0);
    rclcpp::spin_some(node);

    for (size_t index = 0; index < i + 1; index++) {
      currently_selected.push_back(all_state_discipline_one.at(index));
    }
    currently_selected.push_back(23);
    auto data = generate_messages(currently_selected);
    auto result = manage_run(node, data);
    ASSERT_EQ(
      result.back().data,
      2) << "Expected car to transition into MANUAL_MODE = 2, Failed in: " << i;
    data.clear();
    result.clear();
  }
}

/**
 * Test if the car does not react to invalid status messages for state dr_normal
 */
TEST_F(IntegrationTest, TestDoesNotReactToInvalidMessage)
{
  publish_button_once(node, 0);
  auto data = generate_messages(illegal_transitions_dr_normal);
  auto result = manage_run(node, data);
  for (auto msg : result) {
    ASSERT_EQ(
      msg.data,
      10) << "Expected no reaction to invalid messages, Failed with message: " << msg.data;
  }
}

/**
 * Test the Obstacle Evasion Course
 */
TEST_F(IntegrationTest, TestDisciplineObstacleEvasionCourse)
{
  publish_button_once(node, 1);
  auto data = generate_messages(full_run_with_obstacles);
  auto result = manage_run(node, data);
  std::vector<int> expected{10, 13, 14, 14, 10, 5, 7, 8, 9, 10, 5, 6, 8, 9, 10, 5, 6, 10};
  int i = 0;
  for (auto msg : result) {
    ASSERT_EQ(msg.data, expected.at(i)) << "Failed in: " << i;
    i++;
  }
}

/**
 * Test if manual mode can be reached from every state in discipline two
 */
TEST_F(IntegrationTest, TestTransitIntoManualModeDisciplineTwo)
{
  std::vector<int> currently_selected;
  for (size_t i = 0; i < all_state_discipline_two.size(); i++) {
    // Setup
    currently_selected.clear();
    node = nullptr;
    node = std::make_shared<BasicCupStateMachineNode>();
    rclcpp::spin_some(node);
    publish_button_once(node, 1);
    rclcpp::spin_some(node);
    for (size_t index = 0; index < i + 1; index++) {
      currently_selected.push_back(all_state_discipline_two.at(index));
    }
    currently_selected.push_back(23);
    auto data = generate_messages(currently_selected);
    auto result = manage_run(node, data);
    ASSERT_EQ(
      result.back().data,
      2) << "Expected car to transition into MANUAL_MODE = 2, Failed in: " << i;
    data.clear();
    result.clear();
  }
}

/**
 * Test if the car does not react to invalid status messages for state dr_normal
 */
TEST_F(IntegrationTest, TestDoesNotReactToInvalidMessage2)
{
  publish_button_once(node, 1);
  rclcpp::spin_some(node);
  auto data = generate_messages(illegal_transitions_dr_normal_d_two);
  auto result = manage_run(node, data);
  for (auto msg : result) {
    ASSERT_EQ(
      msg.data,
      10) << "Expected no reaction to invalid messages. Reacted to:  " << msg.data;
  }
}

/**
 * Test if car ignores invalid status message in Park
 */
TEST_F(IntegrationTest, TestDoesNotReactToInvalidMessageInPark)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  auto data = generate_messages(ignore_wrong_park);
  auto result = manage_run(node, data);
  ASSERT_EQ(result.back().data, 15) << "Expected car to stay in PR_SEARCH";
}


/**
 * Test if the car stays in manual mode if other status message arrives
 */
TEST_F(IntegrationTest, TestStaysInManualMode)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  auto data = generate_messages(stays_in_manual_mode);
  auto result = manage_run(node, data);
  ASSERT_EQ(result.back().data, 2) << "Expected car to stay in manual mode";
}

/**
 * Test if the car can shutdown
 */
TEST_F(IntegrationTest, TestEntersShutdown)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  std::vector<int> transit_to_normal_drive{0, 9};
  auto data = generate_messages(transit_to_normal_drive);
  auto result = manage_run(node, data);
  result.clear();
  data.clear();
  auto receiver = std::make_shared<DummyPubSub>();
  publish_error_once(node, 2);
  rclcpp::spin_some(node);
  rclcpp::spin_some(receiver);
  std::vector<int> shutdown{25};
  data = generate_messages(shutdown);
  result = manage_run(node, data);
  ASSERT_EQ(result.back().data, 4) << "Expected car to shutdown";
}

/**
 * Test if there is no reaction to warning. Not yet supported
 */
TEST_F(IntegrationTest, TestIgnoresWarning)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  std::vector<int> transit_to_normal_drive{0};
  auto data = generate_messages(transit_to_normal_drive);
  auto result = manage_run(node, data);
  result.clear();
  auto receiver = std::make_shared<DummyPubSub>();
  publish_error_once(node, 1);
  rclcpp::spin_some(node);
  rclcpp::spin_some(receiver);
  result = manage_run(node, data);
  ASSERT_EQ(result.back().data, 10) << "Expected car to stay in drive";
}

/**
 * Test if the car can return to normal drive from manual mode
 */
TEST_F(IntegrationTest, TestCanReturnFromManualMode)
{
  publish_button_once(node, 1);
  rclcpp::spin_some(node);
  auto data = generate_messages(full_run_with_obstacles);
  auto result = manage_run(node, data);
  ASSERT_EQ(
    result.back().data,
    10) << "Expected car to exit manual mode and return to Normal drive";
}

/**
 * Test if the car reacts to an error in drive subautomat
 */
TEST_F(IntegrationTest, TestReactsToErrorInDrive)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  std::vector<int> transit_to_normal_drive{0};
  auto data = generate_messages(transit_to_normal_drive);
  auto result = manage_run(node, data);
  auto dummy = std::make_shared<DummyPubSub>();
  dummy->publish_error(2);
  wait_for_response(node, dummy);
  ASSERT_EQ(dummy->get_state(), 1) << "Expected car to react to error message in normal drive";
}

/**
 * Test if the car react to error in parking subautomat
 */
TEST_F(IntegrationTest, TestReactsToErrorInPark)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  std::vector<int> transit_to_normal_drive{0, 9};
  auto data = generate_messages(transit_to_normal_drive);
  auto result = manage_run(node, data);
  auto dummy = std::make_shared<DummyPubSub>();
  dummy->publish_error(2);
  wait_for_response(node, dummy);
  ASSERT_EQ(
    dummy->get_state(),
    1) << "Expected car to react to error message in parking subautomat";
}

/**
 * Test if the car reacts to error in overtake subautomat
 */
TEST_F(IntegrationTest, TestReactsToErrorInOvertake)
{
  publish_button_once(node, 1);
  rclcpp::spin_some(node);
  std::vector<int> transit_to_normal_drive{0, 8};
  auto data = generate_messages(transit_to_normal_drive);
  auto result = manage_run(node, data);
  auto dummy = std::make_shared<DummyPubSub>();
  dummy->publish_error(2);
  wait_for_response(node, dummy);
  ASSERT_EQ(dummy->get_state(), 1) << "Expected car to react to error in overtake subautomat";
}

/**
 * Test if the car can enter manual mode
 */
TEST_F(IntegrationTest, TestCanLeaveErrorToManualMode)
{
  publish_button_once(node, 0);
  rclcpp::spin_some(node);
  std::vector<int> transit_to_normal_drive{0};
  auto data = generate_messages(transit_to_normal_drive);
  auto result = manage_run(node, data);
  auto dummy = std::make_shared<DummyPubSub>();
  dummy->publish_error(2);
  wait_for_response(node, dummy);
  dummy->publish_status(23);
  wait_for_response(node, dummy);
  ASSERT_EQ(
    dummy->get_state(),
    2) << "Expected car to leave error and transition in manual mode";
}

/**
 * Test if the can can react to a parking sign
 */
TEST_F(IntegrationTest, TestCanReactToSignMessage)
{
  int state_value{-10};
  publish_button_once(node, 0);
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto subscriber = dummy->create_subscription<std_msgs::msg::Int64>(
    STATE_TOPIC, 10, [&state_value](std_msgs::msg::Int64::SharedPtr msg) {
      state_value = msg->data;
    });
  auto publisher_status = dummy->create_publisher<libpsaf_msgs::msg::StatusInfo>(
    "/status/status_info", 10);
  auto publisher_sign = dummy->create_publisher<libpsaf_msgs::msg::Sign>(
    "/sign_detection/sign", 10);

  libpsaf_msgs::msg::StatusInfo status_msg;
  status_msg.type = 0;
  publisher_status->publish(status_msg);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(35));
  libpsaf_msgs::msg::Sign sign;
  sign.type = 3;
  publisher_sign->publish(sign);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(35));
  rclcpp::spin_some(dummy);
  std::chrono::duration<double> time_to_wait(2);
  while (time_to_wait.count() > 0) {
    rclcpp::spin_some(dummy);
    time_to_wait -= std::chrono::duration<double>(0.1);
  }
  ASSERT_EQ(
    state_value,
    15) << "Expected car to react to sign message and transit into parking search";
}

/**
 * Test if the car ignores a currently invalid sign message
 */
TEST_F(IntegrationTest, TestCanIgnoreSignMessage)
{
  int state_value{-10};
  publish_button_once(node, 0);
  auto dummy = std::make_shared<rclcpp::Node>("dummy_node");
  auto subscriber = dummy->create_subscription<std_msgs::msg::Int64>(
    STATE_TOPIC, 10, [&state_value](std_msgs::msg::Int64::SharedPtr msg) {
      state_value = msg->data;
    });
  auto publisher_status = dummy->create_publisher<libpsaf_msgs::msg::StatusInfo>(
    "/status/status_info", 10);
  auto publisher_sign = dummy->create_publisher<libpsaf_msgs::msg::Sign>(
    "/sign_detection/sign", 10);

  libpsaf_msgs::msg::StatusInfo status_msg;
  status_msg.type = 0;
  publisher_status->publish(status_msg);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);
  libpsaf_msgs::msg::Sign sign;
  sign.type = 0;
  publisher_sign->publish(sign);
  rclcpp::spin_some(dummy);
  rclcpp::spin_some(node);
  // active wait for two seconds
  std::chrono::duration<double> time_to_wait(2);
  while (time_to_wait.count() > 0) {
    rclcpp::spin_some(dummy);
    time_to_wait -= std::chrono::duration<double>(0.1);
  }

  rclcpp::spin_some(dummy);
  ASSERT_EQ(state_value, 10) << "Expected car to stay in current state";
}

/**
 * Test if the update Method publishes when called
 */
TEST_F(IntegrationTest, TestCanReceiveFromUpdateMethod)
{
  auto dummy = std::make_shared<DummyPubSub>();
  while (!dummy->msg_received()) {
    rclcpp::spin_some(node);
    node->update();
    rclcpp::spin_some(dummy);
  }
  ASSERT_EQ(dummy->get_state(), 0) << "Expected car to receive message from update method";
}

/**
 * Generate a random sequence of 5 transition and test the state order
 */
TEST_F(IntegrationTest, TestRandomFiveSequence)
{
  std::vector<int> expected;
  std::vector<std_msgs::msg::Int64> result;
  std::vector<int> transition_sequence;

  generate_random_test_case(node, 5, expected, result, transition_sequence);

  ASSERT_EQ(
    result.size() + 1,
    expected.size()) << "Expected size of result and expected are not equal";
  std::string sequence_expected = "";
  std::string sequence_result = "";
  std::string sequence_transition = "";
  for (size_t i = 0; i < result.size(); i++) {
    int expected_value = expected.at(i + 1);
    int result_value = result.at(i).data;
    sequence_expected += std::to_string(expected_value) + " ";
    sequence_result += std::to_string(result_value) + " ";
    sequence_transition += std::to_string(transition_sequence.at(i)) + " ";
    if (expected_value != result_value) {
      FAIL() << "Expected: " << sequence_expected << " Result: " << sequence_result <<
        "For transition sequence: " << sequence_transition;
    }
  }
}

/**
 * Generate a random sequence of 25 transition and test the state order
 */
TEST_F(IntegrationTest, TestRandomTwentyFiveSequence)
{
  std::vector<int> expected;
  std::vector<std_msgs::msg::Int64> result;
  std::vector<int> transition_sequence;

  generate_random_test_case(node, 25, expected, result, transition_sequence);

  ASSERT_EQ(
    result.size() + 1,
    expected.size()) << "Expected size of result and expected are not equal";
  std::string sequence_expected = "";
  std::string sequence_result = "";
  std::string sequence_transition = "";
  for (size_t i = 0; i < result.size(); i++) {
    int expected_value = expected.at(i + 1);
    int result_value = result.at(i).data;
    sequence_expected += std::to_string(expected_value) + " ";
    sequence_result += std::to_string(result_value) + " ";
    sequence_transition += std::to_string(transition_sequence.at(i)) + " ";
    if (expected_value != result_value) {
      FAIL() << "Expected: " << sequence_expected << " Result: " << sequence_result <<
        "For transition sequence: " << sequence_transition;
    }
  }
}

/**
 * Generate a random sequence of 50 transition and test the state order
 */
TEST_F(IntegrationTest, TestRandomFiftySequence)
{
  std::vector<int> expected;
  std::vector<std_msgs::msg::Int64> result;
  std::vector<int> transition_sequence;

  generate_random_test_case(node, 50, expected, result, transition_sequence);

  ASSERT_EQ(
    result.size() + 1,
    expected.size()) << "Expected size of result and expected are not equal";
  std::string sequence_expected = "";
  std::string sequence_result = "";
  std::string sequence_transition = "";
  for (size_t i = 0; i < result.size(); i++) {
    int expected_value = expected.at(i + 1);
    int result_value = result.at(i).data;
    sequence_expected += std::to_string(expected_value) + " ";
    sequence_result += std::to_string(result_value) + " ";
    sequence_transition += std::to_string(transition_sequence.at(i)) + " ";
    if (expected_value != result_value) {
      FAIL() << "Expected: " << sequence_expected << " Result: " << sequence_result <<
        "For transition sequence: " << sequence_transition;
    }
  }
}

/**
 * Generate a random sequence of 100 transition and test the state order
 */
TEST_F(IntegrationTest, TestRandomHundredSequence)
{
  std::vector<int> expected;
  std::vector<std_msgs::msg::Int64> result;
  std::vector<int> transition_sequence;

  generate_random_test_case(node, 100, expected, result, transition_sequence);

  ASSERT_EQ(
    result.size() + 1,
    expected.size()) << "Expected size of result and expected are not equal";
  std::string sequence_expected = "";
  std::string sequence_result = "";
  std::string sequence_transition = "";
  for (size_t i = 0; i < result.size(); i++) {
    int expected_value = expected.at(i + 1);
    int result_value = result.at(i).data;
    sequence_expected += std::to_string(expected_value) + " ";
    sequence_result += std::to_string(result_value) + " ";
    sequence_transition += std::to_string(transition_sequence.at(i)) + " ";
    if (expected_value != result_value) {
      FAIL() << "Expected: " << sequence_expected << " Result: " << sequence_result <<
        "For transition sequence: " << sequence_transition;
    }
  }
}

/**
 * Generate a random sequence of 500 transition and test the state order
 */
TEST_F(IntegrationTest, TestRandomFiveHundredSequence)
{
  std::vector<int> expected;
  std::vector<std_msgs::msg::Int64> result;
  std::vector<int> transition_sequence;
  generate_random_test_case(node, 500, expected, result, transition_sequence);

  ASSERT_EQ(
    result.size() + 1,
    expected.size()) << "Expected size of result and expected are not equal";
  std::string sequence_expected = "";
  std::string sequence_result = "";
  std::string sequence_transition = "";
  for (size_t i = 0; i < result.size(); i++) {
    int expected_value = expected.at(i + 1);
    int result_value = result.at(i).data;
    sequence_expected += std::to_string(expected_value) + " ";
    sequence_result += std::to_string(result_value) + " ";
    sequence_transition += std::to_string(transition_sequence.at(i)) + " ";
    if (expected_value != result_value) {
      FAIL() << "Expected: " << sequence_expected << " Result: " << sequence_result <<
        "For transition sequence: " << sequence_transition;
    }
  }
}

std::vector<std_msgs::msg::Int64> manage_run(
  std::shared_ptr<BasicCupStateMachineNode> node,
  std::vector<libpsaf_msgs::msg::StatusInfo> data)
{
  auto dummy = std::make_shared<PubSubDummy<libpsaf_msgs::msg::StatusInfo, std_msgs::msg::Int64>>(
    "/status/status_info", STATE_TOPIC, data, node);
  dummy->run();
  while (!dummy->hasFinished()) {
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
  }

  auto result = dummy->getResult();

  return result;
}

void generate_random_test_case(
  std::shared_ptr<BasicCupStateMachineNode> node,
  int n, std::vector<int> & expected,
  std::vector<std_msgs::msg::Int64> & result, std::vector<int> & transition_sequence)
{
  // seed the random number generator
  unsigned int seed = (unsigned int)(std::time(NULL));
  // insert 10 in expected values
  expected.push_back(10);
  // randomly select 0 or 1 for the button value
  int button_value = rand_r(&seed) % 2;
  std::cout << "Button value: " << button_value << std::endl;
  publish_button_once(node, button_value);

  std::vector<int> random_values;
  for (int i = 0; i < n; i++) {
    random_values.push_back(rand_r(&seed) % 26);
  }
  // append 0 at the front of random_values
  random_values.insert(random_values.begin(), 0);

  auto data = generate_messages(random_values);
  result = manage_run(node, data);

  auto valid_transitions = state_table.at(10).at(button_value);

  for (auto value : random_values) {
    transition_sequence.push_back(value);
    // check if value is in valid transitions
    if (valid_transitions.find(value) != valid_transitions.end()) {
      expected.push_back(valid_transitions.at(value));
      valid_transitions = state_table.at(valid_transitions.at(value)).at(button_value);
    } else {
      // Insert the last entry in expected again
      expected.push_back(expected.back());
    }
  }
}


void wait_for_response(
  std::shared_ptr<BasicCupStateMachineNode> node,
  std::shared_ptr<DummyPubSub> dummy)
{
  std::chrono::seconds timeout(40);
  auto start = std::chrono::steady_clock::now();
  while (!dummy->msg_received() && std::chrono::steady_clock::now() - start < timeout) {
    rclcpp::spin_some(dummy);
    rclcpp::spin_some(node);
  }
}

void publish_button_once(std::shared_ptr<BasicCupStateMachineNode> node, int button)
{
  auto dummy_node = std::make_shared<DummyPubSub>();
  dummy_node->publish_button(button);
  rclcpp::spin_some(dummy_node);
  rclcpp::spin_some(node);
}

void publish_error_once(std::shared_ptr<BasicCupStateMachineNode> node, int error)
{
  auto dummy_node = std::make_shared<DummyPubSub>();
  dummy_node->publish_error(error);
  rclcpp::spin_some(dummy_node);
  rclcpp::spin_some(node);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
