/**
 * @file lane_detection_node.cpp
 * @brief implementation of the lane_detection
 * @author PSAF
 * @date 2022-06-01
 */
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>

#include "psaf_lane_detection/lane_detection_node.hpp"
#include "psaf_shared/image_utils.hpp"

LaneDetectionConfig LaneDetectionConfig::fromParameters(rclcpp::Node & node)
{
  LaneDetectionConfig c;

  c.updateFrequency = std::fabs(
    node.declare_parameter<double>(
      "update_frequency",
      c.updateFrequency));
  c.debugOutput = node.declare_parameter<bool>("debug_output", c.debugOutput);
  c.saveImages = node.declare_parameter<bool>("save_images", c.saveImages);
  c.showImages = node.declare_parameter<bool>("show_images", c.showImages);

  c.slWidth = std::abs(node.declare_parameter<int>("sliding_window_width", c.slWidth));
  c.slHeight = std::abs(node.declare_parameter<int>("sliding_window_height", c.slHeight));

  c.binarizerblockSize =
    std::abs(node.declare_parameter<int>("binarizer_block_size", c.binarizerblockSize));
  c.binarizerC = node.declare_parameter<double>("binarizer_c", c.binarizerC);

  return c;
}

std::ostream & operator<<(std::ostream & os, const LaneDetectionConfig & cfg)
{
  os << "==== Configuration ====\n";
  os << "- updateFrequency: " << cfg.updateFrequency << "\n";
  os << "- debugOutput: " << std::boolalpha << cfg.debugOutput << "\n";
  os << "- showImages: " << std::boolalpha << cfg.showImages << "\n";
  os << "- saveImages: " << std::boolalpha << cfg.saveImages << "\n";
  os << "- width: " << cfg.slWidth << "\n";
  os << "- height: " << cfg.slHeight << "\n";
  os << "- binarizerblockSize: " << cfg.binarizerblockSize << "\n";
  os << "- binarizerC: " << cfg.binarizerC << "\n";
  os << "- homography: " << cfg.homography << "\n";
  os << "==== End of Cfg =======";

  return os;
}

LaneDetectionNode::LaneDetectionNode(psaf_shared::ImageSaver * imageSaver)
: LaneDetectionInterface(
    LANE_DETECTION_NODE_NAME,
    NBR_OF_CAMS_RGB,
    CAM_TOPIC_RGB,
    STATE_TOPIC,
    LANE_MARKINGS_TOPIC,
    STOP_LINE_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
), mImageSaver(imageSaver)
{
}

void LaneDetectionNode::applyConfig(const LaneDetectionConfig & cfg)
{
  mConfig = cfg;

  // recalculate homography
  mConfig.homography = cv::Mat(3, 3, CV_64F, mConfig.homographyData);

  if (mConfig.debugOutput) {
    std::cout << mConfig << std::endl;
  }
}

LaneDetectionConfig LaneDetectionNode::getConfig() const
{
  return mConfig;
}

void LaneDetectionNode::processImage(cv::Mat & img, int sensor)
{
  // only process color image
  if (sensor == 0 && !img.empty()) {
    // Get image dimensions
    int width = img.cols;
    int height = img.rows;

    // convert rgb color image to opencv bgr image
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

    // Resize image if it's not 640x480
    if (width != 640 || height != 480) {
      cv::resize(img, img, cv::Size(640, 480));
    }

    cv::Mat gray, binary, transformed;
    grayscaleImage(img, gray);
    binarizeImage(gray, binary);
    transformImage(binary, mConfig.homography, transformed);

    if (!(mConfig.showImages || mConfig.saveImages || mConfig.debugOutput)) {
      // if we are not debugging use simple call
      extractLaneMarkings(transformed);
    } else {
      // generate debug image if we are debugging
      auto imgCopy = transformed.clone();
      cv::cvtColor(imgCopy, imgCopy, cv::COLOR_GRAY2BGR);
      extractLaneMarkings(transformed, &imgCopy);
      generateDebugOutput(imgCopy);
    }
  } else {
    if (mConfig.debugOutput) {
      std::cout << "Got empty image or from wrong sensor" << std::endl;
    }
  }
}

void LaneDetectionNode::update()
{
  // Publish Lane Markings information
  if (m_lane_markings.size() >= 3U) {
    publishLaneMarkings(
      m_lane_markings.at(0), m_lane_markings.at(1),
      m_lane_markings.at(2), false, true, 0);
  }
}

void LaneDetectionNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  this->state = state->data;
}

void LaneDetectionNode::grayscaleImage(const cv::Mat & img, cv::Mat & result)
{
  if (img.empty()) {
    if (mConfig.debugOutput) {
      std::cout << "Image is empty" << std::endl;
    }
    return;
  }

  // Check if image has only one channel. If true, do not convert to grayscale
  if (img.channels() == 1) {
    result = img.clone();
  } else {
    cv::cvtColor(img, result, cv::COLOR_BGR2GRAY);
  }
}

void LaneDetectionNode::binarizeImage(const cv::Mat & img, cv::Mat & result)
{
  if (img.empty()) {
    if (mConfig.debugOutput) {
      std::cout << "Image is empty" << std::endl;
    }
    return;
  }

  // use adptiveThreshold, it can go some way solve the problem of light reflection and work better
  cv::adaptiveThreshold(
    img, result, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
    mConfig.binarizerblockSize, mConfig.binarizerC);
}

void LaneDetectionNode::transformImage(
  const cv::Mat & img, const cv::Mat & homography,
  cv::Mat & result)
{
  if (img.empty()) {
    if (mConfig.debugOutput) {
      std::cout << "Image is empty" << std::endl;
    }
    return;
  }

  // check if homography is valid (not empty and 3x3)
  if (homography.empty() || homography.rows != 3 || homography.cols != 3) {
    if (mConfig.debugOutput) {
      std::cout << "Homography is empty or not 3x3" << std::endl;
    }
    return;
  }

  cv::warpPerspective(img, result, homography, img.size());
}

void LaneDetectionNode::resizeImage(const cv::Mat & img, cv::Mat & result)
{
  if (img.empty()) {
    if (mConfig.debugOutput) {
      std::cout << "Input Image is empty" << std::endl;
    }
    return;
  }
  if (img.cols == 640 && img.rows == 480) {
    result = img.clone();
  } else {
    cv::resize(img, result, cv::Size(640, 480), cv::INTER_LINEAR);
  }
}

bool LaneDetectionNode::getMaxContour(const cv::Mat & image, cv::Point & point) const
{
  bool valid = false;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  if (contours.size() > 0U) {
    auto maxContour = std::max_element(
      contours.begin(), contours.end(), [&](const std::vector<cv::Point> & a,
      const std::vector<cv::Point> & b) -> bool {
        return cv::contourArea(a) < cv::contourArea(b);
      });

    const auto moment = cv::moments(*maxContour);
    if (moment.m00 != 0) {
      int cx = moment.m10 / moment.m00;
      int cy = moment.m01 / moment.m00;
      point = cv::Point(cx, cy);
      valid = true;
    }
  }

  return valid;
}

void LaneDetectionNode::extractLaneMarkings(const cv::Mat & img, cv::Mat * debugImage)
{
  // only process non empty image
  if (!img.empty() && img.size().width == 640 && img.size().height == 480 && img.channels() == 1) {
    // try to find base points in image
    auto base_points = getBasePoints(img, debugImage);

    if (base_points.size() > 0U) {
      // information about base point location and wheter to search them
      auto bpInfo = getBasePointInformation(base_points, img.size());

      // Create a vector for the left, center and right lane points
      std::vector<cv::Point> temp_left_points, temp_center_points, temp_right_points;
      // save the number of pixels in the first 3 sliding windows
      unsigned int sum_nonzero_left = 0, sum_nonzero_center = 0, sum_nonzero_right = 0;
      // count windows and empty center windows
      int window_count = 0, empty_center_windows = 0;
      // flags to indicate that we should still add to lanes
      bool left_is_ok = true, center_is_ok = true, right_is_ok = true;

      // run all sliding windows in parallel
      for (int y = 400; y > 40; y -= mConfig.slHeight) {
        // find left lane points
        if (bpInfo.searchL) {
          // limit lbase to image size
          bpInfo.lBase = std::max(
            mConfig.slWidth, std::min(
              bpInfo.lBase,
              img.cols - mConfig.slWidth));

          // get search ranges
          cv::Range l_window_y(y - mConfig.slHeight, y);
          cv::Range l_window_x(bpInfo.lBase - mConfig.slWidth, bpInfo.lBase + mConfig.slWidth);

          // create image limited by sliding window
          auto subImg = img(l_window_y, l_window_x);

          // search for nonzero pixels
          if (window_count <= 3) {
            sum_nonzero_left += cv::countNonZero(subImg);
          }

          // try to find a contour in image
          cv::Point max;
          if (getMaxContour(subImg, max) && left_is_ok) {
            int cx = max.x;
            int cy = max.y;
            int window_diff = static_cast<int>(cx - mConfig.slWidth);

            cv::Point toAdd;

            // limit max change
            if (window_diff > 25) {
              toAdd = cv::Point(bpInfo.lBase, y - cy);
            } else {
              toAdd = cv::Point(bpInfo.lBase - mConfig.slWidth + cx, y - cy);
              bpInfo.lBase = bpInfo.lBase - mConfig.slWidth + cx;
            }
            temp_left_points.push_back(toAdd);

          } else {
            // stop searching
            left_is_ok = false;
          }
        }

        // see earchL for comments
        if (bpInfo.searchC) {
          bpInfo.cBase =
            std::max(mConfig.slWidth, std::min(bpInfo.cBase, img.cols - mConfig.slWidth));
          cv::Range c_window_y(y - mConfig.slHeight, y);
          cv::Range c_window_x(bpInfo.cBase - mConfig.slWidth, bpInfo.cBase + mConfig.slWidth);

          if (window_count <= 3) {
            sum_nonzero_center += cv::countNonZero(img(c_window_y, c_window_x));
          }

          cv::Point max;
          if (getMaxContour(img(c_window_y, c_window_x), max) && center_is_ok) {
            int cx = max.x;
            int cy = max.y;
            cv::Point p(bpInfo.cBase - mConfig.slWidth + cx, y - cy);
            temp_center_points.push_back(p);
            bpInfo.cBase = bpInfo.cBase - mConfig.slWidth + cx;
          } else {
            // stop after 3 empty windows, not after one
            empty_center_windows++;
            if (empty_center_windows >= 2) {
              center_is_ok = false;
            }
          }
        }

        // see searchL for comments
        if (bpInfo.searchR) {
          bpInfo.rBase = std::max(
            mConfig.slWidth, std::min(
              bpInfo.rBase, img.cols - mConfig.slWidth));
          cv::Range r_window_y(y - mConfig.slHeight, y);
          cv::Range r_window_x(bpInfo.rBase - mConfig.slWidth, bpInfo.rBase + mConfig.slWidth);

          if (window_count <= 3) {
            sum_nonzero_right += cv::countNonZero(img(r_window_y, r_window_x));
          }

          cv::Point max;
          if (getMaxContour(img(r_window_y, r_window_x), max) && right_is_ok) {
            int cx = max.x;
            int cy = max.y;
            int window_diff = static_cast<int>(cx - mConfig.slWidth);

            cv::Point toAdd;
            if (window_diff > 25) {
              toAdd = cv::Point(bpInfo.rBase, y - cy);
            } else {
              toAdd = cv::Point(bpInfo.rBase - mConfig.slWidth + cx, y - cy);
              bpInfo.rBase = bpInfo.rBase - mConfig.slWidth + cx;
            }
            temp_right_points.push_back(toAdd);

          } else {
            right_is_ok = false;
          }
        }

        // increment window count by one
        window_count++;

        // draw debug windows if required
        if (debugImage != nullptr) {
          auto drawWindow = [&](const int base) -> void {
              cv::Point p1(base - mConfig.slWidth, y - mConfig.slHeight);
              cv::Point p2(base + mConfig.slWidth, y);
              cv::rectangle(*debugImage, p1, p2, cv::Scalar(255, 255, 255));
            };

          if (bpInfo.searchL && left_is_ok) {
            drawWindow(bpInfo.lBase);
          }
          if (bpInfo.searchC && center_is_ok) {
            drawWindow(bpInfo.cBase);
          }
          if (bpInfo.searchR && right_is_ok) {
            drawWindow(bpInfo.rBase);
          }
        }
      }

      // actual output points
      std::vector<cv::Point> left_lane_points, center_lane_points, right_lane_points;

      // decide if temp points are actually the points they represent
      if (bpInfo.searchR) {
        if (sum_nonzero_center > sum_nonzero_right && base_points[0].x < img.cols * 2 / 5) {
          left_lane_points = temp_center_points;
          center_lane_points = temp_right_points;
        } else {
          center_lane_points = temp_center_points;
          right_lane_points = temp_right_points;
        }
      } else if (bpInfo.searchL) {
        if (sum_nonzero_left > sum_nonzero_center) {
          left_lane_points = temp_left_points;
          center_lane_points = temp_center_points;
        }
      } else if (bpInfo.searchC) {
        center_lane_points = temp_center_points;
      }

      // dont publish empty lane markings
      if (left_lane_points.empty() && center_lane_points.empty() && right_lane_points.empty()) {
        if (mConfig.debugOutput) {
          std::cout << "Failed to identify atleast one lane" << std::endl;
        }
      } else {
        m_lane_markings = {left_lane_points, center_lane_points, right_lane_points};
      }
    } else {
      if (mConfig.debugOutput) {
        std::cout << "No base points found in image" << std::endl;
      }
    }
  } else {
    if (mConfig.debugOutput) {
      std::cout << "Input image is empt, not correct size or not binary" << std::endl;
    }
  }
}

std::vector<cv::Point> LaneDetectionNode::getBasePoints(
  const cv::Mat & img,
  cv::Mat * debugImage) const
{
  std::vector<cv::Point> result;

  if (!img.empty()) {
    // try to find base points in image
    for (int x = 0; x < img.cols - 320; x += 64) {
      cv::Range h_window_y(330, 390);
      cv::Range h_window_x(x, x + 64);

      cv::Point basePoint;

      if (getMaxContour(img(h_window_y, h_window_x), basePoint)) {
        basePoint.x += x;
        basePoint.y += 350;
        result.push_back(basePoint);

        // draw rectangle and base point to debug image
        if (debugImage != nullptr) {
          cv::rectangle(
            *debugImage, cv::Point(x, 330), cv::Point(x + 64, 390),
            cv::Scalar(0, 0, 255));
          cv::drawMarker(*debugImage, basePoint, cv::Scalar(0, 0, 255));
        }
      }
    }
  }

  return result;
}

LaneDetectionNode::BasePointInformation LaneDetectionNode::getBasePointInformation(
  const std::vector<cv::Point> & base_points,
  const cv::Size imageSize) const
{
  BasePointInformation info;

  if (base_points.size() == 1) {
    if (base_points.at(0).x > imageSize.width * 2 / 5) {
      info.rBase = base_points.at(0).x;
      info.searchR = true;
    } else {
      info.cBase = base_points.at(0).x;
      info.searchC = true;
    }
  } else if (base_points.size() > 1) {
    // detemine base_points.at(1) whether is rB or not
    if (base_points.at(1).x > imageSize.width * 2 / 5) {
      if (base_points.at(1).x - base_points.at(0).x < 150 &&
        base_points.at(1).x - base_points.at(0).x > 50)
      {
        info.rBase = base_points.at(1).x;
        info.searchR = true;
        info.cBase = base_points.at(0).x;
        info.searchC = true;
      } else {
        info.rBase = base_points.at(1).x;
        info.searchR = true;
        info.cBase = base_points.at(1).x - 120;
        info.searchC = true;
      }
    } else {
      info.lBase = base_points.at(0).x;
      info.searchL = true;
      info.cBase = base_points.at(1).x;
      info.searchC = true;
    }
  }

  return info;
}

bool LaneDetectionNode::generateDebugOutput(const cv::Mat & img) const
{
  bool generated = false;

  if (m_lane_markings.size() == 3U) {
    const auto & ll = m_lane_markings.at(0);
    const auto & lc = m_lane_markings.at(1);
    const auto & lr = m_lane_markings.at(2);

    // create debug images for new trajectory
    if (mConfig.saveImages || mConfig.showImages) {
      auto points = psaf_shared::plot_points(
        {ll, lc, lr},
        {cv::Scalar(255, 255, 255), cv::Scalar(0, 255, 0),
          cv::Scalar(255, 255, 255)});

      if (mConfig.showImages) {
        psaf_shared::wrapped_imshow("LaneMarkings: Points", points);
        psaf_shared::wrapped_imshow("LaneMarkings: BirdView", img);
      }

      if (mConfig.saveImages && mImageSaver != nullptr) {
        mImageSaver->save(points, "points");
        mImageSaver->save(img, "birdview");
      }
      generated = true;
    }

    // output number of points we found
    if (mConfig.debugOutput) {
      std::cout << "Left Lane (n=" << ll.size() << "): " << ll << std::endl;
      std::cout << "Center Lane (n=" << lc.size() << "): " << lc << std::endl;
      std::cout << "Right Lane (n=" << lr.size() << "): " << lr << std::endl;
      generated = true;
    }
  }

  return generated;
}
