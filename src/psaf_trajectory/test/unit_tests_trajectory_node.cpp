/**
 * @file unit_tests_trajectory_node.cpp
 * @brief The unit tests for the trajectory node
 * @author sven.kalmbach
 * @date 2023-12-30
 */
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "psaf_trajectory/trajectory_node.hpp"
#include "include/helper.hpp"

/**
 * @brief This class will setup the TrajectoryNode config for each test
 */
class TrajectoryUnitTest : public ::testing::Test, public TrajectoryNode
{
public:
  /**
   * @brief function called before each TEST_F, applys config
   */
  void SetUp() override
  {
    // setup default config for all tests
    TrajectoryConfig cfg;
    cfg.debugOutput = true;
    cfg.centerLaneOffset = -2.0;
    cfg.rightLaneOffset = 2.0;
    cfg.leftLaneOffset = -5;
    cfg.enableKalman = false;
    applyConfig(cfg);
  }
};

/**
 * @brief Test valid cases of processLaneMarkings()
 */
TEST_F(TrajectoryUnitTest, TestProcessLaneMarkingsValidCases) {
  // flag if a new traj was published
  bool gotNew = false;
  // last published trajectory
  libpsaf_msgs::msg::Trajectory lastResult;

  // setup trajectory listener
  auto node = std::make_shared<rclcpp::Node>("test_traj_node");
  auto trajSub = node->create_subscription<libpsaf_msgs::msg::Trajectory>(
    TRAJECTORY_TOPIC, rclcpp::QoS(10), [&](libpsaf_msgs::msg::Trajectory::SharedPtr trajectory) {
      lastResult = *trajectory;
      gotNew = true;
    });

  // read all trajectory pairs from disk
  auto pairs = trajectory_testhelper::getTrajectoryTestFiles(
    trajectory_testhelper::getTestDataDir(), "TestExpectedTrajectory");
  ASSERT_GT(pairs.size(), 0U) << "Expect at least one test pair";

  // lambda function to test each pair
  auto testAllPairs = [&](const double abs) {
      for (auto & pair : pairs) {
        gotNew = false;
        const auto markings = trajectory_testhelper::parseLaneMarkingsFromFile(pair.lanes);
        auto pubMarkings = std::make_shared<libpsaf_msgs::msg::LaneMarkings>(markings);
        processLaneMarkings(pubMarkings);
        rclcpp::spin_some(node);

        if (gotNew) {
          std::cout << "Comparing: " << pair.lanes << " and " << pair.trajectory << std::endl;
          auto expectedTrajectory = trajectory_testhelper::parseTrajectoryFromFile(pair.trajectory);
          trajectory_testhelper::compareTrajectories(expectedTrajectory, lastResult, abs);
        } else {
          FAIL() << "Expected new calculated trajectory to arrive";
        }
      }
    };

  // ignore state so we always calculate
  auto cfg = getConfig();
  cfg.ignoreState = true;
  cfg.debugOutput = true;
  applyConfig(cfg);
  testAllPairs(1e-5);

  // TODO(sven.kalmbach): find decent way to test same pairs with kalman
  /*
  cfg.enableKalman = true;
  cfg.enableKalman = true;
  // disable outlier detection
  cfg.kMaxErrorPointDistance = DBL_MAX;
  applyConfig(cfg);
  // allow larger errors with kalman filter, since it smooths lanes somewhat
  testAllPairs(1e-3);
  */
}

/**
 * @brief Test edge cases of processLaneMarkings()
 */
TEST_F(TrajectoryUnitTest, TestProcessLaneMarkingsEdgeCases) {
  // create trajectory subscriber
  bool published = false;
  auto node = std::make_shared<rclcpp::Node>("test_traj_node");
  auto trajSub = node->create_subscription<libpsaf_msgs::msg::Trajectory>(
    TRAJECTORY_TOPIC, rclcpp::QoS(10), [&](libpsaf_msgs::msg::Trajectory::SharedPtr trajectory) {
      (void)trajectory;
      published = true;
    });

  // setup a pointvector with valid values
  PointVector validLane;
  for (size_t i = 0; i < 10; ++i) {
    geometry_msgs::msg::Point p;
    p.x = i;
    p.y = i;
    p.z = 0;
    validLane.push_back(p);
  }

  // helper function for publish lanes
  auto getValidLanes = [&]() -> libpsaf_msgs::msg::LaneMarkings::SharedPtr {
      auto validLanes = std::make_shared<libpsaf_msgs::msg::LaneMarkings>();
      validLanes->center_lane = validLane;
      return validLanes;
    };

  // helper function
  auto testPubbed = [&](const bool shouldPub) {
      published = false;
      rclcpp::spin_some(node);
      ASSERT_EQ(published, shouldPub);
    };

  // test no publishing expected with invalid state
  state = 0;
  auto cfg = getConfig();
  cfg.ignoreState = false;
  applyConfig(cfg);
  processLaneMarkings(getValidLanes());
  testPubbed(false);

  // test no pub with empty lanes
  state = 10;
  auto emptyLanes = std::make_shared<libpsaf_msgs::msg::LaneMarkings>();
  processLaneMarkings(emptyLanes);
  testPubbed(false);

  // test min required size no pub
  cfg = getConfig();
  cfg.minVecLength = DBL_MAX;
  applyConfig(cfg);
  processLaneMarkings(getValidLanes());
  testPubbed(false);

  // expect no exception with imshow and published lanes
  cfg = getConfig();
  cfg.minVecLength = 1e-6;
  cfg.showImages = true;
  applyConfig(cfg);
  processLaneMarkings(getValidLanes());
  testPubbed(true);
}

/**
 * @brief Test valid cases of offsetToTrajectory()
 */
TEST_F(TrajectoryUnitTest, TestOffsetToTrajValidCase) {
  // setup offfsets for this test
  constexpr double testOffset = 1;
  auto cfg = getConfig();
  cfg.leftLaneOffset = testOffset;
  cfg.rightLaneOffset = testOffset;
  cfg.centerLaneOffset = testOffset;
  applyConfig(cfg);

  // setup points for this test
  constexpr unsigned int pointCount = 6U;
  PointVector points(pointCount);
  for (size_t i = 0; i < pointCount; ++i) {
    auto & curr = points.at(i);
    curr.x = 0;
    curr.y = i;
    curr.z = 0;
  }

  // list to iterate over all lane types
  static constexpr std::initializer_list<LaneType> laneTypes = {
    LaneType::LEFT_LANE,
    LaneType::CENTER_LANE,
    LaneType::RIGHT_LANE,
  };

  // run calculation for each lane type
  for (const auto type : laneTypes) {
    const auto results = offsetLaneToTrajectory(points, type);
    ASSERT_EQ(results.size(), pointCount - 1) << "Expect one less point in traj then lanes";

    for (size_t i = 0; i < results.size(); ++i) {
      auto & res = results.at(i);
      geometry_msgs::msg::Point expected;
      expected.x = -testOffset;  // x should be set to offset to right
      expected.y = i;  // test y in range
      expected.z = 0;  // z should always be 0

      EXPECT_NEAR(res.x, expected.x, 1e-6) << "Expect result to be equal to precalculated result";
      EXPECT_NEAR(res.z, expected.z, 1e-6) << "Expect result.z to always be 0";
      // accepted offset from 1 is 1.5 or 0.5, since we expect res to be in middle
      EXPECT_NEAR(res.y, expected.y, 0.5) << "Expect result.y to be between both points";
    }
  }
}

/**
 * @brief Test edge cases of offsetToTrajectory()
 */
TEST_F(TrajectoryUnitTest, TestOffsetToTrajEdgeCases) {
  PointVector points;
  // check no exception with empty points
  offsetLaneToTrajectory(points, LaneType::CENTER_LANE);

  // setup some valid test points
  points.resize(4);
  for (size_t i = 0; i < points.size(); ++i) {
    auto & curr = points.at(i);
    curr.x = i;
    curr.y = i;
    curr.z = i;
  }

  // test for exception when undefined lane typ is inputted
  bool gotException = false;
  try {
    offsetLaneToTrajectory(points, static_cast<LaneType>(-123));
  } catch (...) {
    gotException = true;
  }
  EXPECT_TRUE(gotException) << "Expect exception for invalid LaneType";

  // setup testing for calculation errors
  auto cfg = getConfig();
  cfg.minVecLength = DBL_MAX;
  applyConfig(cfg);

  // calculate expect trajectory
  const auto res = offsetLaneToTrajectory(points, LaneType::CENTER_LANE);
  EXPECT_EQ(res.size(), 0U) << "Expect empty trajectory if min vec length is DBL_MAX";
}

/**
 * @brief Test transformation of points to car coordinates works as expected
 */
TEST_F(TrajectoryUnitTest, TestTransformToCar) {
  // setup cfg for this test
  auto currentCfg = getConfig();
  // expect points not to be rotated and y offste of 1
  currentCfg.rotMatData[0] = 1.0;
  currentCfg.rotMatData[1] = 0.0;
  currentCfg.rotMatData[2] = 0.0;
  currentCfg.rotMatData[3] = 1.0;
  currentCfg.transMatData[0] = 0.0;
  currentCfg.transMatData[1] = 1.0;

  // aply test cfg
  applyConfig(currentCfg);

  // generate input points
  PointVector points(5);
  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = i;
    p.y = i;
    p.z = i;
    points.at(i) = p;
  }

  // run calculation
  auto results = transformToCarCoordinates(points);
  ASSERT_EQ(points.size(), results.size()) << "Expect vec size to remain unchanged";

  for (size_t i = 0; i < results.size(); ++i) {
    // calculate result point for rot mat and trans mat
    geometry_msgs::msg::Point expected;
    expected.x = i;
    expected.y = i + 1;
    expected.z = 0;

    const auto & curr = results.at(i);
    EXPECT_NEAR(
      curr.x, expected.x,
      1e-6) << "Expect result point to be equal to precalculated result.";
    EXPECT_NEAR(
      curr.y, expected.y,
      1e-6) << "Expect result point to be equal to precalculated result.";
    EXPECT_NEAR(
      curr.z, expected.z,
      1e-6) << "Expect result point to be equal to precalculated result.";
  }

  // test no exception when empty
  points.clear();
  transformToCarCoordinates(points);
}

/**
 * @brief Test that unused functions from interface dont change node state or cause pub
 */
TEST_F(TrajectoryUnitTest, TestUnused) {
  const auto oldCfg = getConfig();
  const auto oldState = state;

  // create trajectory subscriber
  auto node = std::make_shared<rclcpp::Node>("test_traj_node");
  auto trajSub = node->create_subscription<libpsaf_msgs::msg::Trajectory>(
    TRAJECTORY_TOPIC, rclcpp::QoS(10), [&](libpsaf_msgs::msg::Trajectory::SharedPtr trajectory) {
      (void)trajectory;
      FAIL() << "Dont expect trajectory to be published in TestUnused";
    });

  update();
  EXPECT_EQ(oldState, state) << "Expect state not to change after unused update() call";
  rclcpp::spin_some(node);  // check no traj is published

  auto obstacle = std::make_shared<libpsaf_msgs::msg::Obstacle>();
  processObstacle(obstacle);
  EXPECT_EQ(oldState, state) << "Expect state not to change after processObstacle update() call";
  rclcpp::spin_some(node);  // check no traj is published
}

/**
 * @brief Check that the trajectory node updates its internal state on state changes
 */
TEST_F(TrajectoryUnitTest, TestStateUpdate) {
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
 * @brief Test that the lane choosing function chooses the expected lane
 */
TEST_F(TrajectoryUnitTest, TestLaneChoosingFunction) {
  PointVector smallest(5);
  PointVector inbetween(10);
  PointVector largest(15);
  TrajectoryNode::LaneType choosenLane;

  // test clear cases: one lane is larger than other
  choosenLane = chooseReferenceLane(inbetween, smallest, largest);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::RIGHT_LANE) << "Right lane should be choosen.";

  choosenLane = chooseReferenceLane(inbetween, largest, smallest);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::CENTER_LANE) << "Center lane should be choosen.";

  choosenLane = chooseReferenceLane(largest, inbetween, smallest);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::LEFT_LANE) << "Left lane should be choosen.";

  // test all lanes are the same size
  choosenLane = chooseReferenceLane(largest, largest, largest);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::RIGHT_LANE) << "Right lane should be choosen.";

  // test center and right are same size
  choosenLane = chooseReferenceLane(inbetween, largest, largest);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::RIGHT_LANE) << "Right lane should be choosen.";

  // test left and center are same zie
  choosenLane = chooseReferenceLane(largest, largest, inbetween);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::CENTER_LANE) << "Center lane should be choosen.";

  // test left and right are same size
  choosenLane = chooseReferenceLane(largest, inbetween, largest);
  EXPECT_EQ(choosenLane, TrajectoryNode::LaneType::RIGHT_LANE) << "Right lane should be choosen.";
}
