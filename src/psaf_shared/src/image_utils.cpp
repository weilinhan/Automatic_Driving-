#include "psaf_shared/image_utils.hpp"

void psaf_shared::wrapped_imshow(const std::string & winname, const cv::InputArray & mat, int wait)
{
  try {
    cv::imshow(winname, mat);
    if (wait >= 0) {
      cv::waitKey(wait);
    }
  } catch (...) {
    // do nothing
  }
}

cv::Mat psaf_shared::plot_points(
  const std::vector<std::vector<cv::Point>> & points,
  const std::vector<cv::Scalar> & colors,
  double scale, double offsetY,
  bool flip, int rotate,
  cv::Size size, cv::Scalar bgColor)
{
  if (points.size() == 0 || colors.size() == 0) {
    throw std::runtime_error("plot_points: got empty vectors.");
  }

  if (points.size() != colors.size()) {
    throw std::runtime_error("plot_points: points and colors vector have to be same size.");
  }

  // create base images from parameters
  cv::Mat image(size, CV_8UC3, bgColor);

  for (size_t i = 0; i < points.size(); ++i) {
    // keep track of our scaled points
    std::vector<cv::Point> scaledPoints;

    for (const auto & point : points.at(i)) {
      // create scaled point
      cv::Point scaledPoint(point.x * scale, point.y * scale + size.height * offsetY);
      scaledPoints.push_back(scaledPoint);

      cv::drawMarker(image, scaledPoint, colors.at(i));
    }

    if (!scaledPoints.empty()) {
      // draw a line connecting points
      cv::polylines(image, scaledPoints, false, colors.at(i));
    }
  }

  if (flip) {
    cv::flip(image, image, 0);
  }

  if (rotate >= 0) {
    cv::rotate(image, image, rotate);
  }

  return image;
}

cv::Mat psaf_shared::plot_points(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & points,
  const std::vector<cv::Scalar> & colors,
  double scale, double offsetY,
  bool flip, int rotate,
  cv::Size size, cv::Scalar bgColor)
{
  if (points.size() == 0 || colors.size() == 0) {
    throw std::runtime_error("plot_points: got empty vectors.");
  }

  if (points.size() != colors.size()) {
    throw std::runtime_error("plot_points: points and colors vector have to be same size.");
  }

  std::vector<std::vector<cv::Point>> cv_points;

  // convert geometry_msg points to opencv points
  for (const auto & point_vectors : points) {
    std::vector<cv::Point> new_cv_points;

    for (const auto & point : point_vectors) {
      new_cv_points.push_back(cv::Point(point.x, point.y));
    }

    // also push back empty vectors
    cv_points.push_back(new_cv_points);
  }

  // call actual drawing function
  return plot_points(cv_points, colors, scale, offsetY, flip, rotate, size, bgColor);
}

cv::Mat psaf_shared::plot_points(
  const std::vector<std::vector<cv::Point2f>> & points,
  const std::vector<cv::Scalar> & colors,
  double scale, double offsetY,
  bool flip, int rotate,
  cv::Size size, cv::Scalar bgColor)
{
  if (points.size() == 0 || colors.size() == 0) {
    throw std::runtime_error("plot_points: got empty vectors.");
  }

  if (points.size() != colors.size()) {
    throw std::runtime_error("plot_points: points and colors vector have to be same size.");
  }

  std::vector<std::vector<cv::Point>> points_2d;

  // convert geometry_msg points to opencv points
  for (const auto & point_vectors : points) {
    std::vector<cv::Point> new_cv_points;

    for (const auto & point : point_vectors) {
      new_cv_points.push_back(cv::Point(point.x, point.y));
    }

    // also push back empty vectors
    points_2d.push_back(new_cv_points);
  }

  return plot_points(points_2d, colors, scale, offsetY, flip, rotate, size, bgColor);
}

cv::Mat psaf_shared::plot_value(
  double value, double maxValue, double minValue, bool mirrorXAxis,
  cv::Size size)
{
  // create image with white bg
  cv::Mat image(size, CV_8UC3, psaf_shared::colors::white);

  // Draw the axis
  cv::line(
    image, cv::Point(10, size.height / 2),
    cv::Point(size.width - 10, size.height / 2),
    psaf_shared::colors::black, 2);

  // mark value on the image
  int x_position;

  if (mirrorXAxis) {
    x_position =
      static_cast<int>((value - minValue) / (maxValue - minValue) * (size.width - 20) + 10);
  } else {
    x_position =
      static_cast<int>((maxValue - value) / (maxValue - minValue) * (size.width - 20) + 10);
  }

  cv::line(
    image, cv::Point(x_position, size.height / 2 - 10),
    cv::Point(x_position, size.height / 2 + 10),
    psaf_shared::colors::red, 2);

  cv::putText(
    image, std::to_string(value),
    cv::Point(x_position - 20, size.height / 2 - 20),
    cv::FONT_HERSHEY_SIMPLEX, 0.5,
    psaf_shared::colors::black, 1);

  return image;
}
