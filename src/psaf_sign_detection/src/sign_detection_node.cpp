/**
 * @file sign_detection_node.cpp
 * @brief the implementation of the SignDetectionNode class
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include "psaf_sign_detection/sign_detection_node.hpp"

#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <sstream>

#include "psaf_sign_detection/sign_id.hpp"
#include "psaf_sign_detection/helpers.hpp"
#include "psaf_shared/image_saver.hpp"
#include "psaf_shared/image_utils.hpp"

SignDetectionConfig SignDetectionConfig::fromParameters(rclcpp::Node & node)
{
  SignDetectionConfig c;
  c.updateFrequency = std::fabs(
    node.declare_parameter<double>("update_frequency", c.updateFrequency));
  c.debugLevel = std::abs(node.declare_parameter<int>("debug_level", c.debugLevel));
  c.ignoreState = node.declare_parameter<bool>("ignore_state", c.debugLevel);

  c.modelFile =
    std::filesystem::path(node.declare_parameter<std::string>("model_file", c.modelFile.string()));
  c.classesFile =
    std::filesystem::path(
    node.declare_parameter<std::string>(
      "classes_file",
      c.classesFile.string()));

  c.confidenceThreshold = std::fabs(
    node.declare_parameter<double>("confidence_threshold", c.confidenceThreshold));
  c.classThreshold = std::fabs(node.declare_parameter<double>("class_threshold", c.classThreshold));
  c.nmsThreshold = std::fabs(node.declare_parameter<double>("nms_threshold", c.nmsThreshold));

  c.allowSignRedetection = node.declare_parameter<bool>(
    "allow_sign_redetection", c.allowSignRedetection);
  c.signLifetime = std::abs(node.declare_parameter<int>("sign_lifetime", c.signLifetime));
  c.triggerDistance =
    std::fabs(node.declare_parameter<double>("trigger_distance", c.triggerDistance));

  c.showImages = node.declare_parameter<bool>("show_images", c.showImages);
  c.saveImages = node.declare_parameter<bool>("save_images", c.saveImages);

  return c;
}

std::ostream & operator<<(std::ostream & os, const SignDetectionConfig & cfg)
{
  os << "==== Configuration ====\n";
  os << "- updateFrequency: " << cfg.updateFrequency << "\n";
  os << "- debugLevel: " << cfg.debugLevel << "\n";
  os << "- ignoreState: " << std::boolalpha << cfg.ignoreState << "\n";
  os << "- modelFile: " << cfg.modelFile << "\n";
  os << "- classesFile: " << cfg.classesFile << "\n";
  os << "- confidenceThreshold: " << cfg.confidenceThreshold << "\n";
  os << "- classThreshold: " << cfg.classThreshold << "\n";
  os << "- nmsThreshold: " << cfg.nmsThreshold << "\n";
  os << "- showImages: " << std::boolalpha << cfg.showImages << "\n";
  os << "- saveImages: " << std::boolalpha << cfg.saveImages << "\n";
  os << "- signLifetime: " << cfg.signLifetime << "\n";
  os << "- triggerDistance: " << cfg.triggerDistance << "\n";
  os << "- allowSignRedetection: " << std::boolalpha << cfg.allowSignRedetection << "\n";
  os << "- distanceHomoGraphyMat: " << cfg.distanceHomoGraphyMat << "\n";
  os << "==== End of Cfg =======";

  return os;
}

SignDetectionNode::SignDetectionNode(psaf_shared::ImageSaver * imageSaver)
: SignDetectionInterface(
    SIGN_DETECTION_NODE_NAME,
    NBR_OF_CAMS,
    CAM_TOPICS,
    STATE_TOPIC,
    SIGN_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
), mImageSaver(imageSaver)
{
}

SignDetectionNode::~SignDetectionNode()
{
  if (mSignDetector != nullptr) {
    delete mSignDetector;
    mSignDetector = nullptr;
  }
}

void SignDetectionNode::applyConfig(const SignDetectionConfig & cfg)
{
  // copy config
  mConfig = cfg;

  // create new sign detector from new config
  // delete old first
  if (mSignDetector != nullptr) {
    delete mSignDetector;
    mSignDetector = nullptr;
  }
  mSignDetector = new SignDetector(
    mConfig.modelFile, mConfig.classesFile,
    mConfig.confidenceThreshold, mConfig.classThreshold,
    mConfig.nmsThreshold);

  // recalculate distance mat
  mConfig.distanceHomoGraphyMat = cv::Mat(3, 3, CV_64F, mConfig.distanceHomographyData);

  // output config if required
  if (mConfig.debugLevel > 0) {
    std::cout << mConfig << std::endl;
  }

  // check the sign detectors model is valid
  if (mSignDetector != nullptr) {
    sign_id::verifyModel(mSignDetector->getClassNames());
  }
}

SignDetectionConfig SignDetectionNode::getConfig() const
{
  return mConfig;
}


void SignDetectionNode::processImage(cv::Mat & img, int sensor)
{
  // only process images if we are driving or we ignore states
  if (state == 10 || mConfig.ignoreState) {
    // process color images only
    if (sensor == 0) {
      // check image is expected format
      helpers::expectedColorFormat(img);
      // convert rgb camera image to opencv bgt by swapping r and b channels
      cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
      mColorImage = img;
    }
  }
}

cv::Point SignDetectionNode::getSignDistancePoint(const SignDetector::Sign & sign) const
{
  return cv::Point(sign.boundingBox.br().x - sign.boundingBox.width / 2, sign.boundingBox.br().y);
}

double SignDetectionNode::calculateDistance(
  const SignDetector::Sign & sign) const
{
  // setup bottom middle point of sign bounding box
  cv::Mat pointMat(3, 1, CV_64F);
  const auto point = getSignDistancePoint(sign);
  pointMat.at<double>(0) = point.x;
  pointMat.at<double>(1) = point.y;
  pointMat.at<double>(2) = 1;

  // calculate point in car coordinates
  cv::Mat resultMat = mConfig.distanceHomoGraphyMat * pointMat;
  resultMat.at<double>(0) /= resultMat.at<double>(2);
  resultMat.at<double>(1) /= resultMat.at<double>(2);

  // only take x distance
  auto distance = resultMat.at<double>(0);

  // negative distances only occur with invalid inputs
  // set to max value so we dont trigger
  if (distance < 0) {
    distance = DBL_MAX;
  }

  return distance;
}

void SignDetectionNode::update()
{
  // only process images in driving or if we ignore states
  if (state == 10 || mConfig.ignoreState) {
    // only proceed if we have both images
    if (!mColorImage.empty()) {
      // create counter for procesing duration
      unsigned int elapsedMs = 0;

      // detect signs using yolo
      if (mSignDetector != nullptr) {
        auto signs = mSignDetector->detectSigns(mColorImage, &elapsedMs);
        // log sign count and duration
        if (mConfig.debugLevel > 1) {
          std::cout << "Detected " << signs.size() << " signs in image, processing took " <<
            elapsedMs << "ms." << std::endl;
        }

        // check each sign individually if we should publish it
        for (const auto & sign : signs) {
          // create a msg instance which to be formated by processSign
          libpsaf_msgs::msg::Sign msg;

          // if we got a msg, we publish it, logging is done in process sign
          if (processSign(sign, calculateDistance(sign), msg)) {
            publishSign(msg);
          }
        }

        // call drawing function if we save or show signs
        if (mConfig.showImages || mConfig.saveImages) {
          drawSignsToFrame(signs, mColorImage);
        }

        // reset member mats to empty
        mColorImage = cv::Mat();

      } else {
        throw std::runtime_error("Sign Detector not initalized, make sure a config is loaded");
      }

    } else {
      if (mConfig.debugLevel > 1) {
        std::cout << "Skipping sign detection (no images saved)" << std::endl;
      }
    }
  } else {
    if (mConfig.debugLevel > 1) {
      std::cout << "Skipping sign detection (state != DRIVING)" << std::endl;
    }
  }
}

bool SignDetectionNode::processSign(
  const SignDetector::Sign & sign, const unsigned int distance,
  libpsaf_msgs::msg::Sign & msg)
{
  // return value
  bool publish = false;
  // reason for not publishing
  std::string reason;

  // check that we are under trigger distance
  if (distance <= mConfig.triggerDistance) {
    // check for previous detections
    if (mDetections.count(sign.id) != 0) {
      // if already detected check if we allow that
      if (mConfig.allowSignRedetection) {
        // if we allow that check for timeout
        if (std::chrono::system_clock::now() >
          mDetections.at(sign.id) + std::chrono::milliseconds(mConfig.signLifetime))
        {
          // if lifetime is over, publish again
          publish = true;
        } else {
          reason = "redetection lifetime still running";
        }
      } else {
        reason = "redetections not allowed";
      }
    } else {
      // if not yet detected, publish
      publish = true;
    }
  } else {
    reason = "not under trigger distance";
  }

  // generate message to publish
  if (publish) {
    // update detection time
    mDetections[sign.id] = std::chrono::system_clock::now();
    // convert our id to libpsaf ids
    msg.type = signIdToPsafId(sign.id);
    // set msg position, y is distance from front of car in mm
    msg.position.x = 0;
    msg.position.y = distance;
    msg.position.z = 0;
  }

  // log if we publish or if we dont, why we wont
  if (mConfig.debugLevel > 0) {
    if (publish) {
      std::cout << "Publishing " << sign;
    } else {
      std::cout << "Not Publishing " << sign << ": " << reason;
    }
    std::cout << std::endl;
  }

  return publish;
}

void SignDetectionNode::drawSignsToFrame(
  const std::vector<SignDetector::Sign> & signs, cv::Mat & colorImage) const
{
  // check input is as expected
  helpers::expectedColorFormat(colorImage);

  // dont save empty images, save some space
  bool signToSave = signs.size() > 0;

  // constant drawing options
  constexpr cv::HersheyFonts font = cv::FONT_HERSHEY_PLAIN;
  constexpr float tScale = 1.2;
  constexpr int tThickness = 1;
  constexpr int tPadding = 10;
  constexpr int bbWidth = 2;

  // constant colors
  const cv::Scalar white(255, 255, 255);
  const cv::Scalar red(0, 0, 255);
  const cv::Scalar black(0, 0, 0);
  const cv::Scalar bbDark(UINT16_MAX, UINT16_MAX, UINT16_MAX);

  for (const auto & sign : signs) {
    // generate box label for each sign
    std::stringstream label;
    label << sign.name() << " c=" << std::setprecision(3) << sign.confidence;
    auto const distance = calculateDistance(sign);
    label << "d=" << std::setprecision(3) << distance;

    // decide rectangle color based on distance
    auto const & rectangleColor = distance < mConfig.triggerDistance ? red : white;

    // draw bounding box, red if under trigger distance
    cv::rectangle(colorImage, sign.boundingBox, rectangleColor, bbWidth);

    cv::drawMarker(
      colorImage,
      cv::Point(
        sign.boundingBox.br().x - sign.boundingBox.width / 2,
        sign.boundingBox.br().y),
      rectangleColor, cv::MARKER_TILTED_CROSS, 20, 3
    );

    // calculate backgroud for text
    cv::Rect textBackground(
      cv::Point2d(sign.boundingBox.x, sign.boundingBox.y),
      cv::getTextSize(label.str(), font, tScale, tThickness, nullptr)
    );

    // acount for padding
    textBackground.width += tPadding; textBackground.height += tPadding;
    // make sure full rectangle is displayed inside image
    helpers::moveToFit(textBackground, colorImage.size());

    // draw the background
    cv::rectangle(colorImage, textBackground, rectangleColor, cv::FILLED);

    // draw text into background
    cv::putText(
      colorImage, label.str(),
      cv::Point(
        textBackground.x + tPadding / 2,
        textBackground.y + (textBackground.height - tPadding) / 2 + tPadding),
      font, tScale, black, tThickness);
  }

  // show images if enabled
  if (mConfig.showImages) {
    psaf_shared::wrapped_imshow("SignDetection: Output", colorImage);
  }

  // save image if enabled
  if (mConfig.saveImages && signToSave && mImageSaver != nullptr) {
    mImageSaver->save(colorImage, "signs");
  }
}

void SignDetectionNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  // store state we got from callback
  this->state = state->data;
  // log state update
  if (mConfig.debugLevel > 1) {
    std::cout << "Got new state: " << this->state << std::endl;
  }
}
