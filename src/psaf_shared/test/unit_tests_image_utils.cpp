/**
 * @file unit_tests.cpp
 * @brief The unit tests for the psaf shared image util functions
 * @author sven.aklmbach
 * @date 2024-02-22
 */
#include <filesystem>

#include "gtest/gtest.h"
#include "psaf_shared/image_utils.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "geometry_msgs/msg/point.hpp"

/**
 * @brief Try to get the path of the TEST_DATA_DIR env variable
 * @return std::filesystem::path path if it exists, else an exception is thrown
 */
std::filesystem::path getTestDataDir()
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

// test that the wrapped imshow call prevents an exception when running headless
TEST(ImageUtilsUnitTests, TestWrappedImshowNoException)
{
  bool caughtException = false;
  cv::Mat testImage(cv::Size(640, 480), CV_8UC3, cv::Scalar(255, 255, 255));

  // test without waitKey
  try {
    psaf_shared::wrapped_imshow("TestWrappedImshowNoException", testImage, -1);
  } catch (...) {
    caughtException = true;
  }

  EXPECT_FALSE(caughtException) << "Expect no exception to be thrown.";
  caughtException = false;

  // test for waitKey to
  try {
    psaf_shared::wrapped_imshow("TestWrappedImshowNoException", testImage, 10);
  } catch (...) {
    caughtException = true;
  }

  EXPECT_FALSE(caughtException) << "Expect no exception to be thrown.";
}

// check if two matrices are equal
static bool areEqual(const cv::Mat & first, const cv::Mat & second)
{
  cv::Mat result;
  cv::bitwise_xor(first, second, result);
  // reshape into single channel for multi channel images
  return !(cv::countNonZero(result.reshape(1)));
}

TEST(ImageUtilsUnitTests, TestPlotValue) {
  auto basePath = getTestDataDir();

  auto notFlipped = psaf_shared::plot_value(0.7, -1.0, 1.0, false, cv::Size(600, 100));
  auto flipped = psaf_shared::plot_value(-0.7, -1.0, 1.0, true, cv::Size(600, 100));

  auto notFlippedResult = cv::imread(basePath / "TestPlotValueImgNotFlipped.png");
  auto flippedResult = cv::imread(basePath / "TestPlotValueImgFlipped.png");

  EXPECT_TRUE(
    areEqual(
      notFlipped,
      notFlippedResult)) << "Expected generated image to be identical to test image";
  EXPECT_TRUE(
    areEqual(
      flipped,
      flippedResult)) << "Expected generated flipped image to be identical to test flipped image";
}

// template function to call plot points quicker for TestPlotPointsError
template<typename T>
bool callPlotPoints(const T & points, const std::vector<cv::Scalar> colors)
{
  bool caughtException = false;

  try {
    psaf_shared::plot_points(points, colors);
  } catch (...) {
    caughtException = true;
  }

  return caughtException;
}

// test error cases for all three function calls
TEST(ImageUtilsUnitTests, TestPlotPointsError) {
  // create three empty and non empty point vectors
  const std::vector<std::vector<cv::Point>> cvPointsEmpty;
  const std::vector<std::vector<geometry_msgs::msg::Point>> geometryPointsEmpty;
  const std::vector<std::vector<cv::Point2f>> cvPointsFEmpty;
  const std::vector<std::vector<cv::Point>> cvPoints(1);
  const std::vector<std::vector<geometry_msgs::msg::Point>> geometryPoints(1);
  const std::vector<std::vector<cv::Point2f>> cvPointsF(1);
  const std::vector<std::vector<cv::Point>> cvPoints2(2);
  const std::vector<std::vector<geometry_msgs::msg::Point>> geometryPoints2(2);
  const std::vector<std::vector<cv::Point2f>> cvPointsF2(2);

  // create empty and non empty color vector
  const std::vector<cv::Scalar> colorsEmpty;
  const std::vector<cv::Scalar> colorsNotEmpty(1);

  EXPECT_TRUE(
    callPlotPoints(
      cvPointsEmpty,
      colorsNotEmpty)) << "Expect exception to be thrown with empty point vector (cvPoints).";
  EXPECT_TRUE(
    callPlotPoints(
      geometryPointsEmpty,
      colorsNotEmpty)) << "Expect exception to be thrown with empty point vector (geometryPoints).";
  EXPECT_TRUE(
    callPlotPoints(
      cvPointsFEmpty,
      colorsNotEmpty)) << "Expect exception to be thrown with empty point vector (cvPointsF).";

  EXPECT_TRUE(
    callPlotPoints(
      cvPoints,
      colorsEmpty)) << "Expect exception to be thrown with empty color vector (cvPoints).";
  EXPECT_TRUE(
    callPlotPoints(
      geometryPoints,
      colorsEmpty)) << "Expect exception to be thrown with empty color vector (geometryPoints).";
  EXPECT_TRUE(
    callPlotPoints(
      cvPointsF,
      colorsEmpty)) << "Expect exception to be thrown with empty color vector (cvPointsF).";

  EXPECT_TRUE(
    callPlotPoints(
      cvPoints2,
      colorsNotEmpty)) << "Expect exception to be thrown with different size vectors (cvPoints).";
  EXPECT_TRUE(
    callPlotPoints(
      geometryPoints2,
      colorsNotEmpty)) <<
    "Expect exception to be thrown with different size vectors (geometryPoints).";
  EXPECT_TRUE(
    callPlotPoints(
      cvPointsF2,
      colorsNotEmpty)) << "Expect exception to be thrown with different size vectors (cvPointsF).";
}

TEST(ImageUtilsUnitTests, TestPlotPointsSameResults) {
  geometry_msgs::msg::Point x1, x2;
  x1.x = 5; x2.x = 10;
  x1.y = 5; x2.y = 10;
  x1.z = 0; x2.z = 0;

  // create the same point vectors three times
  const std::vector<std::vector<geometry_msgs::msg::Point>> geometryPoints = {{x1, x2}};
  const std::vector<std::vector<cv::Point>> cvPoints = {{cv::Point(x1.x, x1.y), cv::Point(
      x2.x,
      x2.y)}};
  const std::vector<std::vector<cv::Point2f>> cvPointsF = {{cv::Point(x1.x, x1.y), cv::Point(
      x2.x,
      x2.y)}};

  const cv::Scalar white(255, 255, 255);

  auto res1 = psaf_shared::plot_points(cvPoints, {white});
  auto res2 = psaf_shared::plot_points(cvPointsF, {white});
  auto res3 = psaf_shared::plot_points(geometryPoints, {white});

  EXPECT_TRUE(areEqual(res1, res2)) << "Expected all generated images to be the same";
  EXPECT_TRUE(areEqual(res2, res3)) << "Expected all generated images to be the same";
}

// test generated image is the same as expected
TEST(ImageUtilsUnitTests, TestPlotPointsExpected) {
  auto basePath = getTestDataDir();

  const std::vector<std::vector<cv::Point>> cvPoints = {{cv::Point(100, 100), cv::Point(300, 300)}};
  auto testImage = psaf_shared::plot_points(cvPoints, {cv::Scalar(255, 255, 255)});
  auto testImageFlipRotate = psaf_shared::plot_points(
    cvPoints,
    {cv::Scalar(255, 255, 255)},
    1.0, 0.0, true,
    cv::ROTATE_90_CLOCKWISE);

  auto expectedImage = cv::imread(basePath / "TestPlotPointsExpected.png");
  auto expectedImageFlipRotate = cv::imread(basePath / "TestPlotPointsExpectedFlipRotate.png");

  EXPECT_TRUE(
    areEqual(
      testImage,
      expectedImage)) << "Expected generated image to be identical to test image";
  EXPECT_TRUE(
    areEqual(
      expectedImageFlipRotate,
      expectedImageFlipRotate)) << "Expected generated image to be identical to test image";
}
