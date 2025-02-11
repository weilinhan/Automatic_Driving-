/**
 * @file unit_tests.cpp
 * @brief unit tests for controller node
 * @author phillip.ziser & sven.kalmbach
 * @date 2024-03-02
 */
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "psaf_controller/controller_node.hpp"
#include "libpsaf_msgs/msg/trajectory.hpp"

/**
 * @brief Class used to test protected functions of ControllerNode
 */
class ControllerUnitTest : public ::testing::Test, public ControllerNode
{
public:
  /**
   * @brief function called before each TEST_F, initializes the nodes config
   */
  void SetUp() override
  {
    auto cfg = ControllerConfig();
    cfg.debugOutput = true;
    cfg.ignoreState = false;
    applyConfig(cfg);

    // reset internal state
    state = 0;
    m_got_first_trajectory = false;
    for (auto & sign : m_active_signs) {
      sign = false;
    }
    m_got_first_sign = false;
  }
};

/**
 * @brief Test whether state handeling works correct
 * @details State 10 is used because it's the typical driving state DR_NORMAL defined in psaf_state_machine/state_definitions.hpp
 */
TEST_F(ControllerUnitTest, TestStateUpdate) {
  // get initial and new state
  state = 0;
  const auto initialState = state;
  const auto newState = 10;

  // create msg format
  const auto pubState = std::make_shared<std_msgs::msg::Int64>();
  pubState.get()->data = newState;

  // update node state and check is as expected
  updateState(pubState);
  EXPECT_EQ(state, newState) << "Expect state to change to new value";
  EXPECT_NE(state, initialState) << "Expect state to not be initial value";
}

/**
 * @brief Tests whether the steering angle is set to the right direction for random trajectories that are only on one side
 * @details A positive y-Value of the target point and a positive angle are seen as left.
 * Note that the uCBridge uses negative y-Values for left.
 */
TEST_F(ControllerUnitTest, TestCalculateSteeringAngle)
{
  // set x and y range to realistic values in cm
  int xRange = 200;
  int yRange = 100;
  int lookaheadRange = 200;

  // create target point
  cv::Point2f point;

  // only create positive x value
  point.x = random() % xRange;
  // create y values on a range [-yRange, yRange]
  point.y = (random() % (2 * yRange)) - yRange;

  const double ld = random() % lookaheadRange;
  const double angle = calculateSteeringAngle(point, ld);

  // the sign of the angle must be the same as the sign of the y value
  EXPECT_EQ(std::signbit(point.y), std::signbit(angle)) <<
    "Expected sign of y value of target point (" << point.x << "," << point.y <<
    ") and sign of steering angle " << angle << " to be equal";
}

/**
 * @brief Test that the controller node publishes a speed value
 */
TEST_F(ControllerUnitTest, TestSpeedPublishing)
{
  unsigned int messageCount = 0;
  // create a speed subscriber
  auto node = std::make_shared<rclcpp::Node>("test_speed_node");
  auto speedSub = node->create_subscription<std_msgs::msg::Int16>(
    "uc_bridge/set_motor_level_forward", rclcpp::QoS(
      10), [&](std_msgs::msg::Int16::SharedPtr p) {
      (void)p;
      ++messageCount;
    });

  // setup node so it updates
  state = 10;
  m_got_first_trajectory = true;

  for (size_t i = 0; i < 3U; i++) {
    // update the node which should result in published speed values
    update();
    // spin subscriber so it can update
    rclcpp::spin_some(node);
  }

  EXPECT_GE(messageCount, 3U) << "Expected that speed is published 3 or more times";
}

/**
 * @brief Test that the controller node publishes a steering value
 */
TEST_F(ControllerUnitTest, TestSteeringPublishing)
{
  unsigned int messageCount = 0;
  // create a steering subscriber
  auto node = std::make_shared<rclcpp::Node>("test_steering_node");
  auto steeringSub = node->create_subscription<std_msgs::msg::Int16>(
    "/uc_bridge/set_steering", rclcpp::QoS(
      10), [&](std_msgs::msg::Int16::SharedPtr p) {
      (void)p;
      ++messageCount;
    });

  state = 10;
  m_got_first_trajectory = true;

  // call node 3 times
  for (size_t i = 0; i < 3U; i++) {
    // generate random test trajectory
    auto traj = std::make_shared<libpsaf_msgs::msg::Trajectory>();
    geometry_msgs::msg::Point p1;
    p1.x = random();
    p1.y = random();
    traj->points.push_back(p1);

    // send traj to controller
    processTrajectory(traj);
    // spin node to update subscriber
    rclcpp::spin_some(node);
  }

  EXPECT_GE(messageCount, 3U) << "Expected that steering angle is published 3 or more times";
}

/**
 * @brief Test that simple callbacks dont cause state change or publishes
 */
TEST_F(ControllerUnitTest, TestSimpleCallbacks)
{
  auto node = std::make_shared<rclcpp::Node>("test_speed_node");
  auto speedSub = node->create_subscription<std_msgs::msg::Int16>(
    "uc_bridge/set_motor_level_forward", rclcpp::QoS(
      10), [&](std_msgs::msg::Int16::SharedPtr p) {
      (void)p;
      FAIL() << "Dont expect speed publish in this test";
    });
  auto steeringSub = node->create_subscription<std_msgs::msg::Int16>(
    "/uc_bridge/set_steering", rclcpp::QoS(
      10), [&](std_msgs::msg::Int16::SharedPtr p) {
      (void)p;
      FAIL() << "Dont expect steering publish in this test";
    });
  const auto oldState = state;

  updateSteering(std::make_shared<std_msgs::msg::Int16>());
  rclcpp::spin_some(node);
  EXPECT_EQ(state, oldState) << "Expect state to remain unchanged()";

  updateSpeed(std::make_shared<std_msgs::msg::Int16>());
  rclcpp::spin_some(node);
  EXPECT_EQ(state, oldState) << "Expect state to remain unchanged()";

  processStopLine(std::make_shared<libpsaf_msgs::msg::StopLine>());
  rclcpp::spin_some(node);
  EXPECT_EQ(state, oldState) << "Expect state to remain unchanged()";
}

/**
 * @brief Test the implemented sign behaviour works as expected
 */
TEST_F(ControllerUnitTest, TestSignBehaviour) {
  // helper lambda to send sign
  auto sendSign = [&](const uint8_t sign) {
      auto msg = std::make_shared<libpsaf_msgs::msg::Sign>();
      msg->type = sign;
      this->processSign(msg);
    };

  auto cfg = getConfig();
  cfg.enableSigns = false;
  cfg.signTimeout = UINT_MAX;
  cfg.targetSpeed = 40;
  applyConfig(cfg);

  // send stop sign with enable signs off
  sendSign(9U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    cfg.targetSpeed) << "Expect target speed because signs are disabled";

  // enable sign behaviour
  cfg.enableSigns = true;
  applyConfig(cfg);

  // send stop sign again
  sendSign(9U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    0U) << "Expect speed 0 because stop sign is enabled";

  // check timeout var works
  EXPECT_FALSE(timeoutActiveSigns()) << "Expect no signs to timeout since timeout is UINT_MAX";
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    0U) << "Expect speed 0 because stop sign is enabled";

  // enable timeouts after 1 ms
  cfg.signTimeout = 0;
  applyConfig(cfg);

  EXPECT_TRUE(timeoutActiveSigns()) << "Expect stop sign to timeout";
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    cfg.targetSpeed) << "Expect target speed because no signs are active";

  sendSign(100U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    cfg.targetSpeed) << "Expect target speed because no handled sign is active";

  // test combined sign behaviour
  // prio3 (priority)
  sendSign(10U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    cfg.targetSpeed + 30) << "Expect target speed +30 for priority";

  // prio 2 (yield)
  sendSign(11U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    40) << "Expect speed 40 for yield with our targetSpeed of 40";

  // prio 1 (30 start)
  sendSign(0U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    40) << "Expect speed 40 for yield + stop with our targetSpeed of 40";

  // prio 1 (stop)
  sendSign(9U);
  EXPECT_EQ(
    getSpeedFromActiveSigns(),
    0) << "Expect speed 0 if stop is active";
}

/**
 * @brief Test that update function resepects state and got first trajectory
 */
TEST_F(ControllerUnitTest, TestUpdateConditions) {
  bool gotSpeed = false;
  auto node = std::make_shared<rclcpp::Node>("test_speed_node");
  auto speedSub = node->create_subscription<std_msgs::msg::Int16>(
    "uc_bridge/set_motor_level_forward", rclcpp::QoS(
      10), [&](std_msgs::msg::Int16::SharedPtr p) {
      // data 0 is ok as controller should always send it if we are not driving
      std::cout << p->data << std::endl;
      gotSpeed = p->data != 0;
    });

  // helper function to check for callback
  auto expectSpeed = [&](const bool expect) {
      gotSpeed = false;
      // need to spin node multiple times since controller publishes
      // multiple times
      rclcpp::spin_some(node);
      rclcpp::spin_some(node);
      EXPECT_EQ(gotSpeed, expect);
    };

  // setup config for this test
  auto cfg = getConfig();
  cfg.ignoreState = false;
  cfg.saveImages = true;
  cfg.showImages = true;
  cfg.enableSigns = true;
  applyConfig(cfg);

  // no state and no ignore --> no publish
  update();
  expectSpeed(false);

  // ignore but no traj--> no publish
  cfg.ignoreState = true;
  applyConfig(cfg);
  update();
  expectSpeed(false);

  // got first point --> expect speed
  m_got_first_trajectory = true;
  update();
  expectSpeed(true);
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  if (!rclcpp::ok()) {
    throw std::runtime_error("Expected ROS to be ok");
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
