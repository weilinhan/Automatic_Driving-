/**
 * @file unit_tests_sign_detection_node.cpp
 * @brief The unit tests for the sign detection node
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include "gtest/gtest.h"

#include "psaf_sign_detection/sign_detection_node.hpp"

/**
 * @brief Try to get the path of the TEST_DATA_DIR env variable
 * @return std::filesystem::path path if it exists, else an exception is thrown
 */
static std::filesystem::path getTestDataDir()
{
  char * ptr = std::getenv("TEST_DATA_DIR");

  if (ptr == nullptr) {
    throw std::runtime_error(
            "TEST_DATA_DIR environment variable not found,"
            " make sure it is set to the resources folder.");
  }

  std::filesystem::path resources(ptr);

  if (!std::filesystem::exists(resources)) {
    throw std::runtime_error(
            "Path specified by TEST_DATA_DIR does not exist (path=" +
            std::string(resources) + ")");
  }

  return resources;
}

// check if two matrices are equal
static bool areEqual(const cv::Mat & first, const cv::Mat & second)
{
  cv::Mat result;
  cv::bitwise_xor(first, second, result);
  // reshape into single channel for multi channel images
  return !(cv::countNonZero(result.reshape(1)));
}

/**
 * @brief Class used to test protected functions of SignDetectionNode
 */
class SignDetectionNodeUnitTests : public ::testing::Test, public SignDetectionNode
{
  void SetUp() override
  {
    // apply fresh config to node for each run
    auto validCfg = SignDetectionConfig();
    // set to max debug Level
    validCfg.debugLevel = 3;
    // load correct paths
    const auto model_path = getTestDataDir().parent_path().parent_path() / "model";
    validCfg.classesFile = model_path / "classes.txt";
    validCfg.modelFile = model_path / "yolov5-7signs.onnx";
    // actually apply the configs
    applyConfig(validCfg);
  }
};

// test update state function stores new state
TEST_F(SignDetectionNodeUnitTests, TestUpdateState) {
  // get initial and new state
  const auto initialState = state;
  const auto newState = initialState + 1;

  // create msg format
  const auto pubState = std::make_shared<std_msgs::msg::Int64>();
  pubState.get()->data = newState;

  // update node state and check is as expected
  updateState(pubState);
  EXPECT_EQ(state, newState) << "Expect state to change to new value";
  EXPECT_NE(state, initialState) << "Expect state to not be initial value";
}

// test image store callback works as expected
TEST_F(SignDetectionNodeUnitTests, TestStoresImage) {
  cv::Mat validImage(cv::Size(640, 480), CV_8UC3, cv::Scalar(123, 123, 123));
  cv::Mat invalidImage(cv::Size(640, 480), CV_16UC1);

  mColorImage = cv::Mat();

  // test that image is only stored in state 10 or if we ignore state
  state = 0;
  mConfig.ignoreState = false;

  processImage(validImage, 0);
  ASSERT_TRUE(mColorImage.empty()) << "Expect valid image not to be stored in state 0";

  state = 10;
  processImage(validImage, 1);
  ASSERT_TRUE(mColorImage.empty()) << "Expect valid image not to be stored if is from wrong sensor";

  state = 0;
  mConfig.ignoreState = true;
  processImage(validImage, 0);
  ASSERT_TRUE(!mColorImage.empty()) << "Expect valid image to be stored if state is ignored";
  EXPECT_TRUE(areEqual(validImage, mColorImage)) << "Expect valid image to be stored";

  bool gotException = false;
  try {
    processImage(invalidImage, 0);
  } catch (...) {
    gotException = true;
  }

  EXPECT_TRUE(gotException) << "Expect exception for invalid color image format";
}

// test that sign processing behaviour resepects config values
TEST_F(SignDetectionNodeUnitTests, TestProcessSign) {
  // setup distance for first tests
  auto cfg = getConfig();
  cfg.allowSignRedetection = false;
  cfg.triggerDistance = 10;
  applyConfig(cfg);

  // create our test sign and message
  libpsaf_msgs::msg::Sign msg;
  SignDetector::Sign testSign(sign_id::SignId::Stop, 0.99, cv::Rect());

  EXPECT_FALSE(
    processSign(
      testSign, cfg.triggerDistance * 2,
      msg)) << "Sign over trigger distance should not be published";
  ASSERT_TRUE(
    processSign(
      testSign, cfg.triggerDistance / 2,
      msg)) << "Sign under trigger distance should be published";
  EXPECT_EQ(
    msg.type, sign_id::signIdToPsafId(testSign.id)) <<
    "Expect message to contain correct sign id when published";

  ASSERT_FALSE(
    processSign(
      testSign, cfg.triggerDistance / 2,
      msg)) << "Expect sign not to be published again without redetections enabled";

  cfg.allowSignRedetection = true;
  cfg.signLifetime = 0;
  applyConfig(cfg);
  ASSERT_TRUE(
    processSign(
      testSign, cfg.triggerDistance / 2,
      msg)) << "Expect sign to be published again when enabled";

  cfg.signLifetime = 10000;
  applyConfig(cfg);
  ASSERT_FALSE(
    processSign(
      testSign, cfg.triggerDistance / 2,
      msg)) << "Expect sign not to be published again when lifetime is still running";

  testSign.id = sign_id::SignId::Priority;
  ASSERT_TRUE(
    processSign(
      testSign, cfg.triggerDistance / 2,
      msg)) << "Expect other sign to be published regardless of previous different sign";
  EXPECT_EQ(
    msg.type, sign_id::signIdToPsafId(testSign.id)) <<
    "Expect message to contain correct sign id when published";
}

// test generates expect output and doesnt cause errors
// no try catch required because test fails upon exception anyways
TEST_F(SignDetectionNodeUnitTests, TestDrawSignsToFrame) {
  // create test image data
  cv::Mat frame(cv::Size(1280, 720), CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<SignDetector::Sign> signs = {
    SignDetector::Sign(sign_id::SignId::PedestrianCrossing, 0.4, cv::Rect(0, 0, 100, 100)),
    SignDetector::Sign(sign_id::SignId::Priority, 0.8, cv::Rect(200, 200, 100, 100)),
    SignDetector::Sign(sign_id::SignId::Stop, 0.9, cv::Rect(300, 300, 100, 100)),
  };

  // draw to frame
  drawSignsToFrame(signs, frame);

  // load expected result
  const auto res = cv::imread(getTestDataDir() / "TestDrawSignsToFrame.png");
  ASSERT_TRUE(!res.empty()) << "Expect expected result to not be empty";

  EXPECT_TRUE(areEqual(frame, res)) << "Expect result image to be equal to expected result";
}

// compare functions results against two manually calculated results for default homography
TEST_F(SignDetectionNodeUnitTests, TestCalculateDistance)
{
  // calculate result of ~80 for default homography
  SignDetector::Sign validSign(sign_id::SignId::PedestrianCrossing, 0,
    cv::Rect(620, 200, 120, 120));
  // calculate result is negative for default homography, should be dbl_max
  SignDetector::Sign invalidSign(sign_id::SignId::PedestrianCrossing, 0,
    cv::Rect(0, 0, 1, 1));

  EXPECT_NEAR(calculateDistance(validSign), 80, 1e-1) << "Expect valid sign distance to be near 80";
  EXPECT_NEAR(
    calculateDistance(invalidSign), DBL_MAX,
    1e-1) << "Expect invalid sign distance to be near DBL_MAX";
}

// test the behaviour of the update function is as we expect:
// - only process with correct preconditions
// - publish single or multiple signs
// - reset image after call so no redetections on same img
TEST_F(SignDetectionNodeUnitTests, TestUpdateBehaviour) {
  // load result image for publish call
  const auto multiSignImage = cv::imread(getTestDataDir() / "TestDetectsSigns7.png");
  constexpr unsigned int expectSignCount = 7;

  // setup config for following tests
  auto cfg = getConfig();
  cfg.allowSignRedetection = true;
  cfg.confidenceThreshold = 0.2;
  cfg.signLifetime = 0;
  cfg.ignoreState = true;
  cfg.triggerDistance = INT_MAX;  // trigger for each detection, regardless of distance
  applyConfig(cfg);

  // setup subscriber to inc count for each sign
  unsigned int pubCount = 0;
  auto sub = std::make_shared<rclcpp::Node>("test_update_node");
  auto signSub = sub->create_subscription<libpsaf_msgs::msg::Sign>(
    SIGN_TOPIC, rclcpp::QoS(10), [&](libpsaf_msgs::msg::Sign::SharedPtr sign) -> void {
      (void)sign;
      ++pubCount;
    });

  // test no publish with empty Mat (ignoring state)
  mColorImage = cv::Mat();
  update();
  rclcpp::spin_some(sub);
  EXPECT_EQ(pubCount, 0U) << "Expect no signs to be published";

  // test no publish when image not empty but state rules apply
  state = 0;
  cfg.ignoreState = false;
  applyConfig(cfg);
  mColorImage = multiSignImage;
  update();
  rclcpp::spin_some(sub);
  EXPECT_TRUE(
    areEqual(
      mColorImage,
      multiSignImage)) << "Expect image to not be modified if not processing";
  EXPECT_EQ(pubCount, 0U) << "Expect no signs to be published";

  state = 10;
  update();
  // spin node atleast n signs time, we only get one callback per spin
  for (unsigned int i = 0; i < expectSignCount + 3; ++i) {
    rclcpp::spin_some(sub);
  }
  EXPECT_EQ(pubCount, expectSignCount) << "Expect no signs to be published";
  EXPECT_TRUE(mColorImage.empty()) << "Expect image to be reset after update called";
}
