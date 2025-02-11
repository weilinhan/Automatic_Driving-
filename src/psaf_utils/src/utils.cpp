/**
 * @file utils.cpp
 * @brief A file containing various helper functions for the development
 * @author PSAF
 * @date 2022-06-01
 */
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <opencv4/opencv2/opencv.hpp>

// Definition if constants
const double EulerConstant = std::exp(1.0);
const double M_PI = 3.14159265358979323846;

/**
 * Method to generate a straight street
 * @return vector of vectors of points containing the points of the lanemarkings
 */
std::vector<std::vector<cv::Point>> generate_straight()
{
  cv::Mat image(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat org(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  int y_index = 100;
  int width, height;
  width = image.cols;
  height = image.rows;
  std::vector<cv::Point> left;
  std::vector<cv::Point> center;
  std::vector<cv::Point> right;
  std::vector<std::vector<cv::Point>> all_points;
  bool skip = false;
  int counter = 0;
  for (int x = 0; x < width; x++) {
    y_index = 0;
    cv::circle(org, cv::Point(y_index + 10, x), 4, cv::Scalar(255, 255, 255), -1);
    cv::circle(org, cv::Point(y_index + 450, x), 4, cv::Scalar(255, 255, 255), -1);
    if (skip) {
      if (counter < 100) {
        counter++;
      } else {
        skip = false;
        counter = 0;
      }
    } else {
      if (counter < 100) {
        cv::circle(org, cv::Point(y_index + 220, x), 4, cv::Scalar(255, 255, 255), -1);
        counter++;
      } else {
        skip = true;
        counter = 0;
      }
    }

    cv::Point left_point(x, y_index + 10);
    cv::Point center_point(x, y_index + 220);
    cv::Point right_point(x, y_index + 450);
    left.push_back(left_point);
    center.push_back(center_point);
    right.push_back(right_point);
  }

  all_points.push_back(left);
  all_points.push_back(center);
  all_points.push_back(right);

  // get current source dir
  std::string current_dir = std::string(__FILE__);
  // save the image in the images folder
  cv::imwrite("straight.png", org);
  return all_points;
}

/**
 * Methode to generate a right turn
 * @return vector of vectors of points containing the points of the lane-markings
 */
std::vector<std::vector<cv::Point>> generate_curve_right()
{
  cv::Mat image(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat org(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  int y_index = 100;
  int width, height;
  width = image.cols;
  height = image.rows;
  std::vector<cv::Point> left;
  std::vector<cv::Point> right;
  std::vector<cv::Point> center;
  std::vector<std::vector<cv::Point>> all_points;
  bool skip = false;
  int counter = 0;
  for (int x = 0; x < width; x++) {
    y_index = static_cast<int>(58.0 * pow(2.7182, (-0.015 * static_cast<double>(x))));
    cv::circle(org, cv::Point(y_index + 10, x), 4, cv::Scalar(255, 255, 255), -1);

    cv::circle(org, cv::Point(y_index + 450, x), 4, cv::Scalar(255, 255, 255), -1);
    if (skip) {
      if (counter < 100) {
        counter++;

      } else {
        skip = false;
        counter = 0;
      }
    } else {
      if (counter < 100) {
        cv::circle(org, cv::Point(y_index + 220, x), 4, cv::Scalar(255, 255, 255), -1);
        counter++;
      } else {
        skip = true;
        counter = 0;
      }
    }

    cv::Point left_point(x, y_index + 10);
    cv::Point center_point(x, y_index + 220);
    cv::Point right_point(x, y_index + 450);
    left.push_back(left_point);
    center.push_back(center_point);
    right.push_back(right_point);
  }

  all_points.push_back(left);
  all_points.push_back(center);
  all_points.push_back(right);

  // get current source dir
  std::string current_dir = std::string(__FILE__);

  // save the image in the images folder
  cv::imwrite("right_curve.png", org);
  return all_points;
}

/**
 * Method to generate a left/right turn -> snake street ;)
 * @return vector of vectors of points containing the points of the lane-markings
 */
std::vector<std::vector<cv::Point>> generate_curve_snake()
{
  cv::Mat image(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat org(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  int y_index = 100;
  int width, height;
  width = image.cols;
  height = image.rows;
  std::vector<cv::Point> left;
  std::vector<cv::Point> right;
  std::vector<cv::Point> center;
  std::vector<std::vector<cv::Point>> all_points;
  bool skip = false;
  int counter = 0;
  for (int x = 0; x < width; x++) {
    y_index = static_cast<int>(30.0 * sin(static_cast<double>(x) / 100.));

    cv::circle(org, cv::Point(y_index + 10, x), 4, cv::Scalar(255, 255, 255), -1);

    cv::circle(org, cv::Point(y_index + 450, x), 4, cv::Scalar(255, 255, 255), -1);

    if (skip) {
      if (counter < 100) {
        counter++;

      } else {
        skip = false;
        counter = 0;
      }
    } else {
      if (counter < 100) {
        cv::circle(org, cv::Point(y_index + 220, x), 4, cv::Scalar(255, 255, 255), -1);
        counter++;
      } else {
        skip = true;
        counter = 0;
      }
    }

    cv::Point left_point(x, y_index + 10);
    cv::Point center_point(x, y_index + 220);
    cv::Point right_point(x, y_index + 450);
    left.push_back(left_point);
    center.push_back(center_point);
    right.push_back(right_point);
  }

  all_points.push_back(left);
  all_points.push_back(center);
  all_points.push_back(right);

  // save the image in the images folder
  cv::imwrite("snake_curve.png", org);
  return all_points;
}

/**
 * Helper function to display a image with a random window name
 * @param[in] image the image to be displayed
 */
void show_image(cv::Mat image)
{
  // Generate a random name for the window
  std::string name = "window" + std::to_string(rand(time(nullptr)));
  // Show the image
  cv::imshow(name, image);
  // Wait for a key to be pressed
  cv::waitKey(0);
  // Destroy the window
  cv::destroyWindow(name);
}

/**
 * Generate a txt file containing the lane marking points. Those files are used as lookup table in
 * the test cases.
 * @param[in] all_points
 */
void generate_output_files_right_curve(std::vector<std::vector<cv::Point>> all_points)
{
  std::cout << "Done!" << std::endl;
  std::vector<cv::Point> right_curve_left = all_points[0];
  std::vector<cv::Point> right_curve_center = all_points[1];
  std::vector<cv::Point> right_curve_right = all_points[2];

  std::cout << "Writing curve to file..." << std::endl;

  // Save the the points in the vector to file
  std::ofstream right_curve_left_file("right_curve_left.txt");
  std::ofstream right_curve_center_file("right_curve_center.txt");
  std::ofstream right_curve_right_file("right_curve_right.txt");

  for (int i = 0; i < right_curve_left.size(); i++) {
    right_curve_left_file << right_curve_left[i].x << " " << right_curve_left[i].y << std::endl;
  }
  for (int i = 0; i < right_curve_center.size(); i++) {
    right_curve_center_file << right_curve_center[i].x << " " << right_curve_center[i].y <<
      std::endl;
  }
  for (int i = 0; i < right_curve_right.size(); i++) {
    right_curve_right_file << right_curve_right[i].x << " " << right_curve_right[i].y << std::endl;
  }

  std::cout << "Done!" << std::endl;
}

/**
 * Generate a txt file containing the lanemarking points. Those files are used as lookup table in
 * the test cases.
 * @param[in] all_points the points
 */
void generate_output_files_snake_curve(std::vector<std::vector<cv::Point>> all_points)
{
  std::cout << "Done!" << std::endl;
  std::vector<cv::Point> snake_curve_left = all_points[0];
  std::vector<cv::Point> snake_curve_center = all_points[1];
  std::vector<cv::Point> snake_curve_right = all_points[2];

  std::cout << "Writing curve to file..." << std::endl;

  // Save the the points in the vector to file
  std::ofstream snake_curve_left_file("snake_curve_left.txt");
  std::ofstream snake_curve_center_file("snake_curve_center.txt");
  std::ofstream snake_curve_right_file("snake_curve_right.txt");

  for (int i = 0; i < snake_curve_left.size(); i++) {
    snake_curve_left_file << snake_curve_left[i].x << " " << snake_curve_left[i].y << std::endl;
  }
  for (int i = 0; i < snake_curve_center.size(); i++) {
    snake_curve_center_file << snake_curve_center[i].x << " " << snake_curve_center[i].y <<
      std::endl;
  }
  for (int i = 0; i < snake_curve_right.size(); i++) {
    snake_curve_right_file << snake_curve_right[i].x << " " << snake_curve_right[i].y << std::endl;
  }

  std::cout << "Done!" << std::endl;
}

/**
 * Generate a txt file containing the lanemarking points. Those files are used as lookup table in
 * the test cases.
 * @param[in] all_points  the points
 */
void generate_output_files_straight(std::vector<std::vector<cv::Point>> all_points)
{
  std::cout << "Done!" << std::endl;
  std::vector<cv::Point> straight_left = all_points[0];
  std::vector<cv::Point> straight_center = all_points[1];
  std::vector<cv::Point> straight_right = all_points[2];

  std::cout << "Writing curve to file..." << std::endl;

  // Save the the points in the vector to file
  std::ofstream straight_left_file("straight_left.txt");
  std::ofstream straight_center_file("straight_center.txt");
  std::ofstream straight_right_file("straight_right.txt");

  for (int i = 0; i < straight_left.size(); i++) {
    straight_left_file << straight_left[i].x << " " << straight_left[i].y << std::endl;
  }
  for (int i = 0; i < straight_center.size(); i++) {
    straight_center_file << straight_center[i].x << " " << straight_center[i].y << std::endl;
  }
  for (int i = 0; i < straight_right.size(); i++) {
    straight_right_file << straight_right[i].x << " " << straight_right[i].y << std::endl;
  }

  std::cout << "Done!" << std::endl;
}

/**
 * Helper function to visualize the points needed to generate a homography.
 * @param[in] image
 */
void build_homography(cv::Mat image)
{
  cv::Point2f src_left_lower = cv::Point(0.0f, 220.0f);
  cv::Point2f src_left_upper = cv::Point(150., 0.);
  cv::Point2f src_right_lower = cv::Point(500., 0.);
  cv::Point2f src_right_upper = cv::Point(640., 220.);

  cv::Point2f dst_left_lower = cv::Point(20., 480.);
  cv::Point2f dst_left_upper = cv::Point(20., 0.);
  cv::Point2f dst_right_lower = cv::Point(620., 0.);
  cv::Point2f dst_right_upper = cv::Point(620., 480.);

  std::vector<cv::Point2f> src_points;
  src_points.push_back(src_left_lower);
  src_points.push_back(src_left_upper);
  src_points.push_back(src_right_lower);
  src_points.push_back(src_right_upper);

  std::vector<cv::Point2f> dst_points;
  dst_points.push_back(dst_left_lower);
  dst_points.push_back(dst_left_upper);
  dst_points.push_back(dst_right_lower);
  dst_points.push_back(dst_right_upper);

  cv::Mat homography = cv::getPerspectiveTransform(src_points, dst_points);

  cv::Mat dst;
  cv::warpPerspective(image, dst, homography, cv::Size(640, 480));

  // save the homography matrix to file in a row major format
  std::ofstream homography_file("homography.txt");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      homography_file << homography.at<double>(i, j) << " ";
    }
    homography_file << std::endl;
  }

  double elements[9] = {1.714285714285715, 1.244155844155844, -237.1428571428572,
    0, 3.98961038961039, 0,
    0, 0.003766233766233766, 1};

  cv::Mat homography_matrix = cv::Mat(3, 3, CV_64F, elements);

  cv::Mat dst_matrix, gray, binary;
  // grayscale image
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  // binary image
  cv::threshold(gray, binary, 180, 255, cv::THRESH_BINARY);

  cv::warpPerspective(binary, dst_matrix, homography_matrix, binary.size());
  cv::imshow("dst_matrix", dst_matrix);
  cv::imshow("image", image);
  cv::imshow("dst", dst);
  cv::waitKey(0);
  cv::imwrite("dst.png", dst);

  cv::imshow("image", image);
  cv::imshow("dst", dst);
  cv::waitKey(0);
}

/**
 * Build a homography and transform the input image
 * @param[in] img the input image
 * @param[out] homography the homography matrix
 * @return the transformed image
 */
cv::Mat build_homography_for_testing(cv::Mat img, cv::Matx<double, 3, 3> & homography)
{
  // grayscale image
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  // binary image
  cv::Mat binary;
  cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
  img = binary.clone();
  cv::Point2f src_left_lower = cv::Point(0.0f, 350.0f);
  cv::Point2f src_left_upper = cv::Point(140., 150.);
  cv::Point2f src_right_lower = cv::Point(400., 150.);
  cv::Point2f src_right_upper = cv::Point(620., 350.);

  cv::Point2f dst_left_lower = cv::Point(20., 480.);
  cv::Point2f dst_left_upper = cv::Point(20., 0.);
  cv::Point2f dst_right_lower = cv::Point(620., 0.);
  cv::Point2f dst_right_upper = cv::Point(620., 480.);

  std::vector<cv::Point2f> src_points;
  src_points.push_back(src_left_lower);
  src_points.push_back(src_left_upper);
  src_points.push_back(src_right_lower);
  src_points.push_back(src_right_upper);

  std::vector<cv::Point2f> dst_points;
  dst_points.push_back(dst_left_lower);
  dst_points.push_back(dst_left_upper);
  dst_points.push_back(dst_right_lower);
  dst_points.push_back(dst_right_upper);

  homography = (cv::getPerspectiveTransform(src_points, dst_points)).clone();


  cv::Mat dst;
  cv::warpPerspective(img, dst, homography, cv::Size(640, 480));

  return dst;
}


/**
 * Test of the Dummy implementation used in the lane detection package.
 * @param[in] img the input image
 * @return the detected lane lines
 */
std::vector<std::vector<cv::Point>> extractLaneMarkings(cv::Mat & img)
{
  std::vector<std::vector<cv::Point>> lane_markings;
  // Get image width and height
  int width = img.cols;
  std::vector<int> y_values{0, 26, 53, 80, 106, 133, 160, 186, 213, 240, 253, 266, 280, 293, 306,
    320, 333, 346, 360, 373, 386, 400, 413, 426, 440, 453, 466};
  // Create a vector for the left, center and right lane points
  std::vector<cv::Point> left_lane_points;
  std::vector<cv::Point> center_lane_points;
  std::vector<cv::Point> right_lane_points;
  for (size_t y = 0; y < y_values.size(); y++) {
    // Get the row from the image depending on the y value
    cv::Mat row = img.row(y_values[y]);
    // Find the non zero pixels in the row
    cv::Mat non_zero_pixels;
    cv::findNonZero(row, non_zero_pixels);
    int sum = 0;
    int count = 0;
    // Iterate over the non zero pixels and average, if the next pixel is a direct neighbor
    for (int x = 0; x < non_zero_pixels.rows - 1; x++) {
      if (non_zero_pixels.at<cv::Point>(x + 1, 0).x - non_zero_pixels.at<cv::Point>(x, 0).x == 1) {
        sum += non_zero_pixels.at<cv::Point>(x, 0).x;
        count++;
      } else {
        if (count == 0) {
          sum = non_zero_pixels.at<cv::Point>(x, 0).x;
          count = 1;
        }
        int average = sum / count;
        if (average < static_cast<int>(width * .18)) {
          left_lane_points.push_back(cv::Point(average, y_values[y]));
        } else if (average > static_cast<int>(width * .18) && average < width / 2) {
          center_lane_points.push_back(cv::Point(average, y_values[y]));
        } else {
          if (count < 80) {
            right_lane_points.push_back(cv::Point(average, y_values[y]));
          }
        }
        sum = 0;
        count = 0;
      }
      if (x == non_zero_pixels.rows - 2) {
        if (count == 0) {
          sum = non_zero_pixels.at<cv::Point>(x, 0).x;
          count = 1;
        }
        int average = sum / count;
        if (average < static_cast<int>(width * .18)) {
          left_lane_points.push_back(cv::Point(average, y_values[y]));
        } else if (average > static_cast<int>(width * .18) && average < width / 2) {
          center_lane_points.push_back(cv::Point(average, y_values[y]));
        } else {
          if (count < 80) {
            right_lane_points.push_back(cv::Point(average, y_values[y]));
          }
        }
        sum = 0;
        count = 0;
      }
    }
  }

  lane_markings.push_back(left_lane_points);
  lane_markings.push_back(center_lane_points);
  lane_markings.push_back(right_lane_points);

  return lane_markings;
}

/**
 * Test function to calculate the lane equation for two points.
 * @param[in] lane_markings the lane markings
 * @param image the input image
 */
void calculate_lane_equation(std::vector<std::vector<cv::Point>> lane_markings, cv::Mat image)
{
  std::vector<cv::Point> right_lane = lane_markings[2];
  // get first point of right lane
  cv::Point right_first_point = right_lane[0];
  // get last point of right lane
  cv::Point right_last_point = right_lane[right_lane.size() - 1];
  // get x and y of the first point and last point and swap them

  double y_1 = static_cast<double>(right_first_point.x);
  double x_1 = static_cast<double>(right_first_point.y);
  double y_2 = static_cast<double>(right_last_point.x);
  double x_2 = static_cast<double>(right_last_point.y);

  // calculate slope and y-intercept
  double m = (y_2 - y_1) / (x_2 - x_1);
  double b = y_1 - m * x_1;

  // create an empty image 640x480

  // Print the lane equation
  std::cout << "Lane equation: y = " << m << "x + " << b << std::endl;
}

/**
 * Sliding window implementation with visualization. This algorithm is used in the test cases
 * to dynamically fine the lane markings
 * @param org the original image
 * @param image a copy of the original image
 * @param width the width of the search window
 * @param height the height of the search window
 * @return the detected lane markings as float
 */
std::vector<cv::Point2f> sliding_window(cv::Mat org, cv::Mat image, int width, int height)
{
  std::vector<cv::Point2f> output_points;
  std::vector<std::vector<cv::Point2f>> lane_markings;
  std::vector<bool> output_points_filled;
  const cv::Size image_size = image.size();
  bool break_method = false;
  std::vector<cv::Point> starting_points;
  // get window width and height
  cv::Rect starting_window = cv::Rect(0, 360, width, 120);
  int starting_window_width = starting_window.width;
  int starting_window_height = starting_window.height;

  // Search for starting points
  for (int x = 0; x <= image.cols - starting_window_width; x += starting_window_width) {
    // Extract the roi based on image
    cv::Mat roi = image(starting_window);
    cv::rectangle(org, starting_window, cv::Scalar(255, 0, 0), 1);
    cv::imshow("Image", org);
    cv::waitKey(32);
    // Count non zero pixels
    int non_zero_pixels = cv::countNonZero(roi);
    // If there are more than half of the pixels in the window,
    // add the starting point to the starting points vector
    if (non_zero_pixels > (starting_window_height * starting_window_width / 128)) {
      starting_points.push_back(cv::Point(x, starting_window.y));
    }
    starting_window.x += starting_window_width;
  }
  // Iterate over the starting points
  std::vector<cv::Point> cleaned_points;
  bool skip = false;
  for (size_t index = 0; index < starting_points.size(); index++) {
    if (skip) {
      skip = false;
      continue;
    }
    cv::Point current = starting_points[index];
    cv::Point next = starting_points[index + 1];
    if (next.x - current.x == width) {
      int average = (current.x + next.x) / 2;
      cleaned_points.push_back(cv::Point(average, image.rows - height));
      skip = true;
    } else {
      cleaned_points.push_back(cv::Point(current.x, image.rows - height));
    }
  }

  // draw the starting points
  std::cout << "Creating search windows" << std::endl;
  std::vector<cv::Rect> search_windows;
  for (size_t i = 0; i < cleaned_points.size(); i++) {
    cv::Point point = cleaned_points[i];
    cv::Rect search_window = cv::Rect(point.x, point.y, width, height);
    search_windows.push_back(search_window);
  }
  std::cout << "Searching for lane markings" << std::endl;
  for (auto window : search_windows) {
    bool should_break = false;
    int skip_counter = 0;
    output_points.clear();
    cv::Mat vis = org.clone();
    while (!should_break) {
      if (window.x + width > image.cols || window.y + height > image.rows || window.x < 0 ||
        window.y < 0)
      {
        should_break = true;
        continue;
      }
      cv::Mat roi = image(window);
      cv::Mat non_zero;
      cv::findNonZero(roi, non_zero);
      // std::cout << "Found non zero pixels" << non_zero.size() << std::endl;
      if (!non_zero.empty()) {
        if (window.y != 0) {
          skip_counter = 0;
        }

        // find average x
        int average_x = 0;
        int average_y = 0;
        for (size_t i = 0; i < non_zero.rows; i++) {
          cv::Point point = non_zero.at<cv::Point>(i, 0);
          average_x += point.x + window.x;
          average_y += point.y + window.y;
        }
        average_x /= non_zero.rows;
        average_y /= non_zero.rows;
        output_points.push_back(cv::Point(average_x, average_y));
      }
      cv::Rect up_left = cv::Rect(window.x - width, window.y - height, width, height);
      cv::Rect up_center = cv::Rect(window.x, window.y - height, width, height);
      cv::Rect up_right = cv::Rect(window.x + width, window.y - height, width, height);
      cv::rectangle(vis, up_left, cv::Scalar(0, 255, 0), 2);
      cv::rectangle(vis, up_center, cv::Scalar(0, 255, 0), 2);
      cv::rectangle(vis, up_right, cv::Scalar(0, 255, 0), 2);

      if (up_left.x < 0 || up_left.y < 0 || up_left.x + width > image.cols ||
        up_left.y > image.rows)
      {
        up_left = cv::Rect(0, 0, 0, 0);
      }
      if (up_center.x < 0 || up_center.y < 0 || up_center.x + width > image.cols ||
        up_center.y > image.rows)
      {
        up_center = cv::Rect(0, 0, 0, 0);
      }
      if (up_right.x < 0 || up_right.y < 0 || up_right.x + width > image.cols ||
        up_right.y > image.rows)
      {
        up_right = cv::Rect(0, 0, 0, 0);
      }
      if (up_left.empty() && up_center.empty() && up_right.empty()) {
        should_break = true;
        continue;

      } else {
        int non_zero_left = 0;
        int non_zero_center = 0;
        int non_zero_right = 0;
        if (!up_left.empty()) {
          cv::Mat roi = image(up_left);
          cv::Mat non_zero;
          cv::findNonZero(roi, non_zero);
          non_zero_left = non_zero.rows;
        }
        if (!up_center.empty()) {
          cv::Mat roi = image(up_center);
          cv::Mat non_zero;
          cv::findNonZero(roi, non_zero);
          non_zero_center = non_zero.rows;
        }
        if (!up_right.empty()) {
          cv::Mat roi = image(up_right);
          cv::Mat non_zero;
          cv::findNonZero(roi, non_zero);
          non_zero_right = non_zero.rows;
        }
        // assign the window with the most non zero pixels to window
        if (non_zero_left > non_zero_center && non_zero_left > non_zero_right) {
          window = up_left;
        } else if (non_zero_center > non_zero_left && non_zero_center > non_zero_right) {
          window = up_center;
        } else if (non_zero_right > non_zero_left && non_zero_right > non_zero_center) {
          window = up_right;
        } else {
          if (skip_counter > 0) {
            should_break = true;
          }
          window.y -= height;
          skip_counter += 1;
        }
        cv::rectangle(vis, window, cv::Scalar(0, 0, 255), 2);
        cv::imshow("Searching for lane markings", vis);
        cv::waitKey(32);
        if (window.y < 0 || window.y + height > image.rows || window.x + width > image.cols ||
          window.x < 0)
        {
          should_break = true;
        }
      }
    }
    lane_markings.push_back(output_points);
  }

  // draw the output points
  for (size_t i = 0; i < lane_markings.size(); i++) {
    for (size_t j = 0; j < lane_markings[i].size(); j++) {
      cv::Point point = lane_markings[i][j];
      cv::circle(org, point, 5, cv::Scalar(0, 0, 255), 2);
    }
  }

  cv::imshow("output", org);
  cv::waitKey(32);
  for (size_t i = 0; i < lane_markings.size(); i++) {
    // remove vector with less than 2 points
    auto current = lane_markings[i];
    if (current.size() < 2) {
      lane_markings.erase(lane_markings.begin() + i);
      i--;
    }
  }
  std::cout << "Lane Markings" << lane_markings.size() << std::endl;
  return output_points;
}

/**
 * OpenCV implementation of the QR-Code detection. Does not work with the OpenCV version used
 * in this workspace.
 * @param[in] image The image to detect the QR-Code in.
 */
void find_qr_code(cv::Mat image)
{
  cv::QRCodeDetector qrDetector = cv::QRCodeDetector::QRCodeDetector();

  cv::Mat bbox, rectifiedImage;

  std::string data = qrDetector.detectAndDecode(image, bbox, rectifiedImage);

  if (data.length() > 0) {
    std::cout << "QR Code detected: " << data << std::endl;
  } else {
    std::cout << "QR Code not detected" << std::endl;
  }
}


int main(int, char **)
{
  // To execute the code add the method to the main function
  return 0;
}
