/**
 * @file unit_tests.cpp
 * @brief unit tests for lane detection node
 * @author sven.kalmbach & PSAF
 * @date 2024-03-02
 */
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <cmath>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "psaf_lane_detection/lane_detection_node.hpp"
#include "opencv4/opencv2/opencv.hpp"

/**
 * @class LaneDetectionNodeTests
 * @brief Testsuite for the unit tests of the lane detection node
 * @details This testsuite contains the unit tests for the lane detection node.
 */
class LaneDetectionUnitTests : public LaneDetectionNode, public ::testing::Test
{
  void SetUp() override
  {
    auto config = LaneDetectionConfig();
    config.debugOutput = true;
    applyConfig(config);

    m_lane_markings.clear();
  }
};

/**
 * @brief Compare two cv::Size instances by their sizes
 * @param a size to compare to b
 * @param b size to compare to a
 * @return true if both widths and heights are equal
 */
bool operator==(const cv::Size & a, const cv::Size & b)
{
  return a.height == b.height && a.width == b.width;
}

/**
 * @brief Compare two cv::Point instances by their sizes
 * @param a point to compare to b
 * @param b point to compare to a
 * @return true if both point coordinates are equal
 */
bool operator==(const cv::Point & a, const cv::Point & b)
{
  return a.x == b.x && a.y == b.y;
}

/**
 * @brief Check if both matrices are equal
 * @param first matrix to compare to second
 * @param second matrix to compare to first
 * @return true if both are equal, else false
 */
static bool areEqual(const cv::Mat & first, const cv::Mat & second)
{
  cv::Mat result;
  cv::bitwise_xor(first, second, result);
  // reshape into single channel for multi channel images
  return !(cv::countNonZero(result.reshape(1)));
}


/**
 * @brief Test the image resizing function works as expected
 */
TEST_F(LaneDetectionUnitTests, TestImageResize)
{
  cv::Mat cameraImage(cv::Size(1280, 720), CV_8UC3);
  resizeImage(cameraImage, cameraImage);
  ASSERT_EQ(cameraImage.size(), cv::Size(640, 480)) << "Expect image to be resized to 640x480";

  resizeImage(cameraImage, cameraImage);
  EXPECT_EQ(
    cameraImage.size(),
    cv::Size(640, 480)) << "Expect with correct size to remain unchanged";

  cv::Mat empty;
  resizeImage(empty, empty);
  EXPECT_TRUE(empty.empty()) << "Expect empty image to remain empty";
}

/**
 * @brief Test image grayscaling
 */
TEST_F(LaneDetectionUnitTests, TestGrayScaleImage)
{
  cv::Mat empty;
  grayscaleImage(empty, empty);
  ASSERT_TRUE(empty.empty()) << "Expect empty image to remain empty";

  cv::Mat resized(cv::Size(640, 480), CV_8UC3);
  grayscaleImage(resized, resized);
  ASSERT_EQ(resized.channels(), 1) << "Expected grayscale image to have only one channel";

  cv::Mat result;
  grayscaleImage(resized, result);
  EXPECT_TRUE(areEqual(resized, result)) << "Expect grayscale image to remain uncahnged";

  std::string base_path = std::string(std::getenv("TEST_DATA_DIR"));
  cv::Mat img, gray, expected;

  // test grayscaling works
  img = cv::imread(base_path + "/color_calib.png");
  expected = cv::imread(base_path + "/color_calib_grey.png", cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(img.empty()) << "Expect test image not to be empty";
  ASSERT_FALSE(expected.empty()) << "Expect test image not to be empty";
  grayscaleImage(img, gray);

  EXPECT_TRUE(areEqual(gray, expected)) << "Expect result image to be grayscaled";
}

/**
 * @brief Test if image transform can handle edge cases
 */
TEST_F(LaneDetectionUnitTests, TestTransformEdgeCases)
{
  const auto cfg = getConfig();
  const auto validHomo = cfg.homography;

  cv::Mat empty;
  transformImage(empty, validHomo, empty);
  EXPECT_TRUE(empty.empty()) << "Expect empty image to remain empty";

  cv::Mat validImage(cv::Size(640, 480), CV_8UC3);
  cv::Mat res, emptyHomo;
  transformImage(validImage, emptyHomo, res);
  EXPECT_TRUE(res.empty()) << "Expect res to be empty, since homography was empty";

  cv::Mat invalidSizeHomo(1, 3, CV_64F);
  transformImage(validImage, invalidSizeHomo, res);
  EXPECT_TRUE(res.empty()) << "Expect res to be empty, since homography size was not 3x3";
}

/**
 * @brief Test transformation with different homographies
 */
TEST_F(LaneDetectionUnitTests, TestDoesTransformCorrectly)
{
  // Create a 3x3 int matrix with entries 1-9
  double input_params[9] =
  {1, 2, 3, 4, 5, 6, 7, 8, 9};

  cv::Mat input = cv::Mat(3, 3, CV_64F, input_params);

  // Define the entries for the homography matrices
  double h_1[9] = {0.0, 1.0, 0.0,
    -1.0, 0.0, 1.0,
    0.0, -0.0, 1.0};

  double h_2[9] = {-1.0, 0.0, 1.0,
    0.0, -1.0, 1.0,
    -0.0, -0.0, 1.0};

  double h_3[9] = {0.0, -1.0, 1.0,
    1.0, 0.0, 0.0,
    -0.0, 0.0, 1.0};

  double h_4[9] = {-1.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    0.0, -0.0, 1.0};

  double h_5[9] = {0.0, 1.0, 0.0,
    -1.0, 0.0, 1.0,
    0.0, -0.0, 1.0};

  // Define the entries for expected output matrices
  double r_1[9] = {2.0, 5.0, 8.0,
    1.0, 4.0, 7.0,
    0.0, 0.0, 0.0};

  double r_2[9] = {5.0, 4.0, 0.0,
    2.0, 1.0, 0.0,
    0.0, 0.0, 0.0};

  double r_3[9] = {4.0, 1.0, 0.0,
    5.0, 2.0, 0.0,
    6.0, 3.0, 0.0};

  double r_4[9] = {2.0, 1.0, 0.0,
    5.0, 4.0, 0.0,
    8.0, 7.0, 0.0, };

  double r_5[9] = {2.0, 5.0, 8.0,
    1.0, 4.0, 7.0,
    0.0, 0.0, 0.0};

  // Create the homography matrices
  cv::Mat homograhpy_1 = cv::Mat(3, 3, CV_64F, h_1);
  cv::Mat homograhpy_2 = cv::Mat(3, 3, CV_64F, h_2);
  cv::Mat homograhpy_3 = cv::Mat(3, 3, CV_64F, h_3);
  cv::Mat homograhpy_4 = cv::Mat(3, 3, CV_64F, h_4);
  cv::Mat homograhpy_5 = cv::Mat(3, 3, CV_64F, h_5);

  // Create the result matrices
  cv::Mat expected_result_1 = cv::Mat(3, 3, CV_64F, r_1);
  cv::Mat expected_result_2 = cv::Mat(3, 3, CV_64F, r_2);
  cv::Mat expected_result_3 = cv::Mat(3, 3, CV_64F, r_3);
  cv::Mat expected_result_4 = cv::Mat(3, 3, CV_64F, r_4);
  cv::Mat expected_result_5 = cv::Mat(3, 3, CV_64F, r_5);

  cv::Mat actual_result_1, actual_result_2, actual_result_3, actual_result_4, actual_result_5;

  // Run the method
  transformImage(input, homograhpy_1, actual_result_1);
  transformImage(input, homograhpy_2, actual_result_2);
  transformImage(input, homograhpy_3, actual_result_3);
  transformImage(input, homograhpy_4, actual_result_4);
  transformImage(input, homograhpy_5, actual_result_5);

  // Create an empty vector to store the results
  std::vector<double> result_1_vec;
  std::vector<double> result_2_vec;
  std::vector<double> result_3_vec;
  std::vector<double> result_4_vec;
  std::vector<double> result_5_vec;

  // Convert the results to vectors
  for (int i = 0; i < actual_result_1.rows; i++) {
    for (int j = 0; j < actual_result_1.cols; j++) {
      result_1_vec.push_back(actual_result_1.at<double>(i, j));
      result_2_vec.push_back(actual_result_2.at<double>(i, j));
      result_3_vec.push_back(actual_result_3.at<double>(i, j));
      result_4_vec.push_back(actual_result_4.at<double>(i, j));
      result_5_vec.push_back(actual_result_5.at<double>(i, j));
    }
  }

  // Create an empty vector to store the expected results
  std::vector<double> expected_result_1_vec;
  std::vector<double> expected_result_2_vec;
  std::vector<double> expected_result_3_vec;
  std::vector<double> expected_result_4_vec;
  std::vector<double> expected_result_5_vec;

  // Convert the expected results to vectors
  for (int i = 0; i < expected_result_1.rows; i++) {
    for (int j = 0; j < expected_result_1.cols; j++) {
      expected_result_1_vec.push_back(expected_result_1.at<double>(i, j));
      expected_result_2_vec.push_back(expected_result_2.at<double>(i, j));
      expected_result_3_vec.push_back(expected_result_3.at<double>(i, j));
      expected_result_4_vec.push_back(expected_result_4.at<double>(i, j));
      expected_result_5_vec.push_back(expected_result_5.at<double>(i, j));
    }
  }

  // Compare the results
  ASSERT_EQ(result_1_vec, expected_result_1_vec) <<
    "Expected result 1 to be equal to actual result 1";
  ASSERT_EQ(result_2_vec, expected_result_2_vec) <<
    "Expected result 2 to be equal to actual result 2";
  ASSERT_EQ(result_3_vec, expected_result_3_vec) <<
    "Expected result 3 to be equal to actual result 3";
  ASSERT_EQ(result_4_vec, expected_result_4_vec) <<
    "Expected result 4 to be equal to actual result 4";
  ASSERT_EQ(result_5_vec, expected_result_5_vec) <<
    "Expected result 5 to be equal to actual result 5";
}

/**
 * @brief Test if the binarization works and handles edge cases
 */
TEST_F(LaneDetectionUnitTests, TestImageBinarization)
{
  cv::Mat empty;
  binarizeImage(empty, empty);
  EXPECT_TRUE(empty.empty()) << "Expected empty input image to result in empty binarized";

  // Create a test matrix as an upper triangular matrix of size 20x20
  cv::Mat img = cv::Mat::zeros(20, 20, CV_8UC1);
  cv::Mat expected = cv::Mat::zeros(20, 20, CV_8UC1);
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      if (i < j) {
        img.at<uchar>(i, j) = 180;
        expected.at<uchar>(i, j) = 255;
      }
    }
  }

  cv::Mat result;
  binarizeImage(img, result);
  ASSERT_TRUE(areEqual(result, expected)) << "Expected vector and result vector did not match";
}

/**
 * @brief Test if the lane extraction handles empty inputs or blank images
 */
TEST_F(LaneDetectionUnitTests, TestExtractLaneMarkingEdgeCases)
{
  cv::Mat img;
  std::vector<std::vector<cv::Point>> result;
  extractLaneMarkings(img);
  EXPECT_TRUE(m_lane_markings.empty()) << "Expected extracted lanes to be empty";

  img = cv::Mat::zeros(480, 640, CV_8UC1);
  extractLaneMarkings(img);
  EXPECT_TRUE(m_lane_markings.empty()) << "Expected extracted lanes to be empty";
}

/**
 * @brief Test if the lane extraction node stores updated state
 */
TEST_F(LaneDetectionUnitTests, TestUpdateState) {
  // get initial and new state
  state = 0;
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

/**
 * @brief Test the get contour function handles edge cases and detects contours
 */
TEST_F(LaneDetectionUnitTests, TestGetContour) {
  cv::Mat empty;
  cv::Point res;
  EXPECT_FALSE(getMaxContour(empty, res)) << "Expect no contour in empty image";

  cv::Mat withContours(cv::Size(40, 40), CV_8UC1, cv::Scalar(0));
  cv::drawMarker(withContours, cv::Point(20, 20), cv::Scalar(255), cv::MARKER_SQUARE);
  ASSERT_TRUE(getMaxContour(withContours, res));
  EXPECT_NEAR(res.x, 20, 3) << "Expect found contour to be near marker";
  EXPECT_NEAR(res.y, 20, 3) << "Expect found contour to be near marker";
}

/**
 * @brief Test that the debug output function doesnt cause an exception
 * regardless of config or lane markings
 */
TEST_F(LaneDetectionUnitTests, TestDebugOutput) {
  auto cfg = getConfig();
  cfg.showImages = true;
  cfg.debugOutput = true;
  applyConfig(cfg);

  // test with empty inputs --> no exceptions
  cv::Mat empty;
  m_lane_markings.clear();
  generateDebugOutput(empty);

  // test with non empty inputs --> no exceptions
  m_lane_markings.push_back({cv::Point(1, 1)});
  empty = cv::Mat(cv::Size(640, 480), CV_8UC3);
  generateDebugOutput(empty);
}


/**
 * @brief Test the base point function correctly identifies base points
 */
TEST_F(LaneDetectionUnitTests, TestGetBasePointInformation) {
  // test empty points
  std::vector<cv::Point> points;
  auto res = getBasePointInformation(points, cv::Size(0, 0));
  EXPECT_FALSE(
    res.searchC && res.searchL &&
    res.searchR) << "Expect no base points in empty vector";

  // check no error if more than three points
  points.resize(100);
  res = getBasePointInformation(points, cv::Size(0, 0));
  points.clear();

  // chekc identifes left lane
  const cv::Size size(640, 480);
  points.push_back(cv::Point(100, 0));
  res = getBasePointInformation(points, size);
  EXPECT_FALSE(res.searchC && res.searchR) << "Expect to only search for center lane";
  EXPECT_TRUE(res.searchC) << "Expect to only search for center lane";
  EXPECT_EQ(res.cBase, 100) << "Expect base point x to be expected value";

  // chekc identifes right lane
  points.at(0) = cv::Point(640, 0);
  res = getBasePointInformation(points, size);
  EXPECT_FALSE(res.searchC && res.searchL) << "Expect to only search for right lane";
  EXPECT_TRUE(res.searchR) << "Expect to only search for right lane";
  EXPECT_EQ(res.rBase, 640) << "Expect base point x to be expected value";

  // TODO(sven.kalmbach) add further tests for multiple points
}

/**
 * @brief Test the base point function correctly finds base points and handles edges
 */
TEST_F(LaneDetectionUnitTests, TestGetBasePoints) {
  cv::Mat img(cv::Size(640, 480), CV_8UC1, cv::Scalar(0));
  cv::Mat debugImage(cv::Size(640, 480), CV_8UC3, cv::Scalar(0, 0, 0));
  auto initialDbg = debugImage.clone();

  // handle empty images
  auto res = getBasePoints(img, nullptr);
  ASSERT_EQ(res.size(), 0U) << "Expect no points in empty image";
  res = getBasePoints(img, &debugImage);
  ASSERT_EQ(res.size(), 0U) << "Expect no points in empty image if using debug image";
  EXPECT_TRUE(
    areEqual(
      debugImage,
      initialDbg)) << "Expect debugImage to remain unchanged if no points are found";

  // find a point in non empty image
  cv::drawMarker(img, cv::Point(50, 350), cv::Scalar(255, 255, 255));
  res = getBasePoints(img, &debugImage);
  ASSERT_EQ(res.size(), 1U) << "Expect single point";
  EXPECT_FALSE(
    areEqual(
      debugImage,
      initialDbg)) << "Expect debugImage to change if points are found";

  // test two points
  cv::drawMarker(img, cv::Point(150, 350), cv::Scalar(255, 255, 255));
  res = getBasePoints(img, &debugImage);
  ASSERT_EQ(res.size(), 2U) << "Expect two point";
  EXPECT_FALSE(
    areEqual(
      debugImage,
      initialDbg)) << "Expect debugImage to chnage if points are found";

  // TODO(sven.kalmbach) add further tests for multiple points
}

/**
 * @brief Test that the update function publishes results if available
 */
TEST_F(LaneDetectionUnitTests, TestUpdatePublishing) {
  bool got = false;
  libpsaf_msgs::msg::LaneMarkings last;

  auto node = std::make_shared<rclcpp::Node>("test_update_node");
  auto laneSub = node->create_subscription<libpsaf_msgs::msg::LaneMarkings>(
    LANE_MARKINGS_TOPIC, rclcpp::QoS(10), [&](
      libpsaf_msgs::msg::LaneMarkings::SharedPtr lanes) -> void {
      got = true;
      last = *lanes;
    });

  // expect no publish with empty markings
  m_lane_markings.clear();
  update();
  rclcpp::spin_some(node);
  EXPECT_FALSE(got) << "Expect no lane markings to be published if they are empty";

  // publish with not valid size m_lane_markings
  got = false;
  m_lane_markings.resize(2);
  update();
  rclcpp::spin_some(node);
  EXPECT_FALSE(got) << "Expect no lane markings to be published if not enoug points";

  // expect correct publish with non empty markings
  m_lane_markings.resize(3);
  m_lane_markings.at(0) = {cv::Point(1, -1)};
  got = false;
  update();
  rclcpp::spin_some(node);
  ASSERT_TRUE(got) << "Expect lane markings to be published if the arent empty";
  // conver geometry msg to cv point
  cv::Point res(last.left_lane.at(0).x, last.left_lane.at(0).y);
  ASSERT_EQ(res, cv::Point(1, -1));
}

/**
 * @brief Test that the processImage callback handles edge cases and causes
 * lane markings to be detected
 */
TEST_F(LaneDetectionUnitTests, TestImageProcessing) {
  cv::Mat empty;
  processImage(empty, 0);
  ASSERT_TRUE(m_lane_markings.empty()) << "Expected no lane markings to be detected in empty image";


  // load image that definetly will contain lane markings
  const auto contains = cv::imread(std::string(std::getenv("TEST_DATA_DIR")) + "/car_image.png");
  auto input = contains.clone();

  // process this image to check if default case works
  processImage(input, 0);
  ASSERT_FALSE(m_lane_markings.empty()) << "Expected lane markings to be detected";
  m_lane_markings.clear();

  // test invalid sensor not publishing
  input = contains.clone();
  processImage(input, 1);
  ASSERT_TRUE(m_lane_markings.empty()) <<
    "Expected no lane markings to be detected with invalid sensor";
  m_lane_markings.clear();

  // test no error when showing images
  auto cfg = getConfig();
  cfg.showImages = true;
  applyConfig(cfg);
  input = contains.clone();
  processImage(input, 0);
  ASSERT_FALSE(m_lane_markings.empty()) << "Expected lane markings to be detected";
}

/**
 * @brief Test that the extractLaneMarking functions handles basic cases
 */
TEST_F(LaneDetectionUnitTests, TestLaneMarkingExtraction) {
  // check no results for empty image
  cv::Mat empty;
  extractLaneMarkings(empty);
  ASSERT_TRUE(m_lane_markings.empty()) << "Expect no lane markings for empty image";

  // check no results for image with wrong size
  cv::Mat wrongSize(cv::Size(100, 100), CV_8UC1, cv::Scalar(255));
  extractLaneMarkings(wrongSize);
  ASSERT_TRUE(m_lane_markings.empty()) << "Expect no lane markings for image with wrong size";

  // check no results for image in wrong format
  cv::Mat wrongFormat(cv::Size(100, 100), CV_8UC3, cv::Scalar(255));
  extractLaneMarkings(wrongFormat);
  ASSERT_TRUE(m_lane_markings.empty()) << "Expect no lane markings for image with wrong format";

  // check no results for black image
  cv::Mat black(cv::Size(640, 480), CV_8UC1, cv::Scalar(0));
  extractLaneMarkings(black);
  ASSERT_TRUE(m_lane_markings.empty()) << "Expect no lane markings for black image";

  // check generates results for single image and debug is written to
  const auto contains = cv::imread(
    std::string(
      std::getenv(
        "TEST_DATA_DIR")) + "/car_image_binary.png", cv::IMREAD_GRAYSCALE);
  auto input = contains.clone();
  // generate debug image
  cv::Mat debugImage = contains.clone();
  cv::cvtColor(debugImage, debugImage, cv::COLOR_GRAY2BGR);
  const auto initialDbg = debugImage.clone();

  // test with car image
  extractLaneMarkings(contains, &debugImage);
  EXPECT_FALSE(m_lane_markings.empty()) << "Expect lane markings for all white image";
  ASSERT_FALSE(areEqual(debugImage, initialDbg)) << "Expect debug image to be generated";
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
