/**
 * @file image_utils.hpp
 * @brief Definition of the util functions for image generation
 * @author PSAF
 * @date 2024-01-29
 */
#ifndef PSAF_SHARED__IMSHOW_UTIL_HPP_
#define PSAF_SHARED__IMSHOW_UTIL_HPP_

#include <string>
#include <vector>

#include "opencv4/opencv2/opencv.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace psaf_shared
{

/**
 * @brief Wrap the cv::imageshow call to prevent uncaught exceptions when running headless
 * @param winname name of the window to show
 * @param mat cv::Mat to show
 * @param wait parameter passed to waitkey, use negative value if no waitkey is required
 */
void wrapped_imshow(const std::string & winname, const cv::InputArray & mat, int wait = 10);

/**
 * @brief Create a image with each point vector connected by marked lines
 * @param points A vector containg multiple point vectors to be drawn
 * @param colors Color for each of the point vectors to be drawn
 * @param scale scale applied to the points, defaults to 1
 * @param offsetY relative y shift: point.y += size.height * offsetY, defaults to 0
 * @param flip whether to flip the generated image
 * @param rotate -1 to not rotate, else use cv::RotateEnum
 * @param size size of the generated image, defaults to 640x480
 * @param bgColor background color of the image, defaults to black
 * @return cv::Mat the generated image
 */
cv::Mat plot_points(
  const std::vector<std::vector<cv::Point>> & points,
  const std::vector<cv::Scalar> & colors,
  double scale = 1, double offsetY = 0, bool flip = false,
  int rotate = -1, cv::Size size = cv::Size(640, 480),
  cv::Scalar bgColor = cv::Scalar(0, 0, 0));

// overloaded method which accepts cv::Point2f
cv::Mat plot_points(
  const std::vector<std::vector<cv::Point2f>> & points,
  const std::vector<cv::Scalar> & colors,
  double scale = 1, double offsetY = 0, bool flip = false,
  int rotate = -1, cv::Size size = cv::Size(640, 480),
  cv::Scalar bgColor = cv::Scalar(0, 0, 0));

// overloaded method which accepts geometry_msgs::msg::Point
cv::Mat plot_points(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & points,
  const std::vector<cv::Scalar> & colors,
  double scale = 1, double offsetY = 0, bool flip = false,
  int rotate = -1, cv::Size size = cv::Size(640, 480),
  cv::Scalar bgColor = cv::Scalar(0, 0, 0));

/**
 * @brief Create a image containg a 1d axis with a value marked
 * @param value the value to mark on the axis
 * @param maxValue max value of the axis
 * @param minValue min value of the axis
 * @param mirrorXAxis whether to mirror the axis or not
 * @param size size of the image, defaults to 600x100
 * @return cv::Mat the generated image
 */
cv::Mat plot_value(
  double value, double maxValue, double minValue,
  bool mirrorXAxis = false, cv::Size size = cv::Size(600, 100));

// color constants used for drawing
namespace colors
{
const cv::Scalar black = cv::Scalar(0, 0, 0);
const cv::Scalar white = cv::Scalar(255, 255, 255);
const cv::Scalar red = cv::Scalar(0, 0, 255);
const cv::Scalar green = cv::Scalar(0, 255, 0);
const cv::Scalar blue = cv::Scalar(255, 0, 0);
const cv::Scalar yellow = cv::Scalar(0, 255, 255);
const cv::Scalar purple = cv::Scalar(255, 0, 255);
const cv::Scalar cyan = cv::Scalar(255, 255, 0);
}  // namespace colors


}  // namespace psaf_shared

#endif  // PSAF_SHARED__IMSHOW_UTIL_HPP_
