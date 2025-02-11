/**
 * @file helpers.cpp
 * @brief the implementation of the helpers methods
 * @author sven.kalmbach
 * @date 2024-02-12
 */
#include "psaf_sign_detection/helpers.hpp"

void helpers::expectedColorFormat(const cv::Mat & image)
{
  if (image.channels() != 3 || image.type() != CV_8UC3) {
    throw std::runtime_error("Unexpected color format!");
  }
}

cv::Rect & helpers::moveToFit(cv::Rect & rectangle, const cv::Size fitTo)
{
  if (rectangle.x < 0) {
    rectangle.x = 0;
  }

  if (rectangle.y < 0) {
    rectangle.y = 0;
  }

  if (rectangle.x + rectangle.width > fitTo.width) {
    rectangle.x -= (rectangle.x + rectangle.width) - fitTo.width;
  }

  if (rectangle.y + rectangle.height > fitTo.height) {
    rectangle.y -= (rectangle.y + rectangle.height) - fitTo.height;
  }

  return rectangle;
}

cv::Rect & helpers::cropToFit(cv::Rect & rectangle, const cv::Size & fitIn)
{
  if (rectangle.x < 0) {
    rectangle.x = 0;
  }

  if (rectangle.y < 0) {
    rectangle.y = 0;
  }

  if (rectangle.x + rectangle.width > fitIn.width) {
    rectangle.width = fitIn.width - rectangle.x;
  }

  if (rectangle.y + rectangle.height > fitIn.height) {
    rectangle.height = fitIn.height - rectangle.y;
  }

  return rectangle;
}
