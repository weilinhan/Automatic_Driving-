/**
 * @file unit_tests_helpers.cpp
 * @brief The unit tests for the helpers namespace
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include "gtest/gtest.h"

#include "psaf_sign_detection/helpers.hpp"

TEST(HelpersUnitTest, TestExpectedColorFormat) {
  const cv::Mat color(cv::Size(640, 480), CV_8UC3);
  const cv::Mat invalid(cv::Size(640, 480), CV_16UC3);

  auto callExpected = [&](const cv::Mat & img) -> bool {
      bool gotException = false;
      try {
        helpers::expectedColorFormat(img);
      } catch (...) {
        gotException = true;
      }
      return gotException;
    };

  EXPECT_TRUE(callExpected(invalid)) << "Expect an exception for invalid image format";
  EXPECT_FALSE(callExpected(color)) << "Expect no exception for correct image format";
}

TEST(HelpersUnitTest, TestMoveToFit) {
  // size all rectangles should fit in
  cv::Size size(640, 480);

  // test rectangles for each edge case
  const cv::Size rectangleSize(50, 50);
  cv::Rect negativeX(cv::Point(-50, 100), rectangleSize);
  cv::Rect negativeY(cv::Point(50, -100), rectangleSize);
  cv::Rect toHigh(cv::Point(50, 450), rectangleSize);
  cv::Rect toWide(cv::Point(630, 100), rectangleSize);
  cv::Rect fits(cv::Point(50, 50), rectangleSize);
  std::vector<cv::Rect> rects = {negativeX, negativeY, toHigh, toWide, fits};

  auto testResult = [&](const cv::Rect & test) {
      // expect size to be unchanged
      EXPECT_EQ(test.size(), rectangleSize) << "Expect rectangle size to remain unchanged.";
      EXPECT_LE(test.x + test.width, size.width) << "Expect rectangle x + width to fit.";
      EXPECT_LE(test.y + test.height, size.height) << "Expect rectangle y + width to fit.";
    };

  for (auto & rect : rects) {
    helpers::moveToFit(rect, size);
    testResult(rect);
  }
}

TEST(HelpersUnitTest, TestCropToFit) {
  // size all rectangles should fit in
  cv::Size size(640, 480);

  // test rectangles for each edge case
  const cv::Size rectangleSize(50, 50);
  cv::Rect negativeX(cv::Point(-50, 100), rectangleSize);
  cv::Rect negativeY(cv::Point(50, -100), rectangleSize);
  cv::Rect toHigh(cv::Point(50, 450), rectangleSize);
  cv::Rect toWide(cv::Point(630, 100), rectangleSize);
  cv::Rect fits(cv::Point(50, 50), rectangleSize);
  std::vector<cv::Rect> rects = {negativeX, negativeY, toHigh, toWide};

  auto testResult = [&](const cv::Rect & test) {
      EXPECT_LE(
        test.size().area(),
        rectangleSize.area()) << "Expect rectangle size to be cropped or unchanged";
      EXPECT_LE(test.x + test.width, size.width) << "Expect rectangle x + width to fit.";
      EXPECT_LE(test.y + test.height, size.height) << "Expect rectangle y + width to fit.";
    };

  for (auto & rect : rects) {
    helpers::cropToFit(rect, size);
    testResult(rect);
  }

  // manually test for rectangle that shouldnt be modifed
  helpers::cropToFit(fits, size);
  EXPECT_EQ(fits.size(), rectangleSize) << "Expect rectangle size to be cropped unchanged.";
  EXPECT_LE(fits.x + fits.width, size.width) << "Expect rectangle x + width to fit.";
  EXPECT_LE(fits.y + fits.height, size.height) << "Expect rectangle y + width to fit.";
}
