/**
 * @file sign_detection_node.hpp
 * @brief The definition of the SignDetectionNode class
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_SIGN_DETECTION__SIGN_DETECTION_NODE_HPP_
#define PSAF_SIGN_DETECTION__SIGN_DETECTION_NODE_HPP_

#include <cmath>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "libpsaf/interface/sign_detection_interface.hpp"
#include "libpsaf_msgs/msg/sign.hpp"

#include "psaf_configuration/configuration.hpp"
#include "psaf_sign_detection/sign_detector.hpp"
#include "psaf_shared/image_saver.hpp"
#include "opencv4/opencv2/opencv.hpp"

/**
 * @brief Struct used to encapsulate SignDetectionNode config
 */
struct SignDetectionConfig
{
  // update frequency of this node
  double updateFrequency = 60.0;
  // verbosity of this nodes debug output 0=disabled, 1-x increments of verbosity
  unsigned int debugLevel = 0;
  // whether to process signs regardless of statemachine state
  bool ignoreState = false;

  // path to the .onnx of the model to use, relative to node launch dir.
  // UPDATE sign_id.hpp IF YOU CHANGE CLASSES!
  std::filesystem::path modelFile = "invalid";
  // path to the file containing the classes, relative to node launch dir
  std::filesystem::path classesFile = "invalid";

  // limit confidence score of detection, if lower than this detection is not considered
  double confidenceThreshold = 0.65;
  // limit confidence of maximum class match, if lower than this no class is assigned to detection
  double classThreshold = 0.5;
  // threshold for filtering out multiple boxes
  double nmsThreshold = 0.4;

  // whether to allow a sign to be detected again after lifteime is over
  bool allowSignRedetection = true;
  // lifetime of signs in milliseconds if redetection is enabled
  unsigned int signLifetime = 10000;
  // distance at which a sign will be published
  double triggerDistance = 100.0;

  // whether to show each processed frame with results
  bool showImages = false;
  // whether to save each processed frame with results
  bool saveImages = false;

  // distance calculation homography values, not loaded from parameters
  double distanceHomographyData[9] {
    9.44376751e-03, -9.48636057e-03, -5.37996855e+01,
    7.07141886e-02, 3.56618483e-03, -4.66725869e+01,
    1.88437827e-04, -5.49353950e-03, 1.0
  };

  // homography matrix used to calculate distance from signs
  cv::Mat distanceHomoGraphyMat = cv::Mat(3, 3, CV_64F, distanceHomographyData);

  /**
   * @brief Load a SignDetectionConfig from parameters using the provided node
   * @param node used to load the config
   * @return SignDetectionConfig the loaded config
   */
  static SignDetectionConfig fromParameters(rclcpp::Node & node);

  /**
   * @brief Output this config to an ostream
   * @param os the ostream to output to
   * @param cfg the config to output to the stream
   * @return std::ostream& reference to the input ostream
   */
  friend std::ostream & operator<<(std::ostream & os, const SignDetectionConfig & cfg);
};

/**
 * @class SignDetectionNode
 * @implements SignDetectionInterface
 * @brief Detect the signs located next to the road.
 * @details This class is the node of the sign detection. It is responsible for
 *          the detection of signs and the publishing of the detected signs.
 */
class SignDetectionNode : public libpsaf::SignDetectionInterface
{
public:
  /**
   * @brief Construct a new Sign Detection Node object
   * @param imageSaver optional pointer to the image save instance to use
   */
  explicit SignDetectionNode(psaf_shared::ImageSaver * imageSaver = nullptr);

  /**
   * @brief Destroy the Sign Detection Node object
   */
  ~SignDetectionNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

  /**
   * @brief Apply the cfg to this node
   * @param cfg the cfg to apply to this node
   */
  void applyConfig(const SignDetectionConfig & cfg);

  /**
   * @brief Get this nodes current SignDetectionConfig
   * @return SignDetectionConfig the current config
   */
  SignDetectionConfig getConfig() const;

protected:
  /**
   * @brief Callback Method for the image topic
   * @param[in] img the image
   * @param[in] sensor the position of the topic in the topic vector
   */
  void processImage(cv::Mat & img, int sensor) final;

  /**
   * @brief Callback Method for the state
   * @param[in] state the current state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Get the point on a sign bounding box used to calculate the distance
   * @param sign the sign we calculate the point from
   * @return cv::Point the point
   */
  cv::Point getSignDistancePoint(const SignDetector::Sign & sign) const;

  /**
   * @brief Calculate the distance of a sign from the car
   * @param sign the sign to calculate the distance of
   * @return double the distance in cm
   */
  double calculateDistance(const SignDetector::Sign & sign) const;

  /**
   * @brief Draw all signs in the sign vector to the color image
   * @details also shows and saves images if flags are set
   * @param signs vector containg signs to be drawn
   * @param colorImage image to draw the signs to
  */
  void drawSignsToFrame(
    const std::vector<SignDetector::Sign> & signs, cv::Mat & colorImage) const;

  /**
   * @brief Check whether we should publish a sign and generate a message for it
   * @details checks for lifetimes and detection thresholds
   * @param sign the sign to check for publishing
   * @param distance distance of this sign from car
   * @param msg msg that needs to be published if return is true
   * @return bool true if msg was formatted and needs to be published else false
   */
  bool processSign(
    const SignDetector::Sign & sign, const unsigned int distance,
    libpsaf_msgs::msg::Sign & msg);

private:
  // config of this node
  SignDetectionConfig mConfig;
  // node sign detector instance
  SignDetector * mSignDetector = nullptr;
  // last recieved color image
  cv::Mat mColorImage;
  // ImageSaver instance used to save debug images
  psaf_shared::ImageSaver * const mImageSaver = nullptr;
  // map used to keep track of previous sign detections
  std::map<sign_id::SignId, std::chrono::time_point<std::chrono::system_clock>> mDetections;

#ifdef BUILD_UNIT_TEST
  friend class SignDetectionNodeUnitTests_TestStoresImage_Test;
  friend class SignDetectionNodeUnitTests_TestUpdateState_Test;
  friend class SignDetectionNodeUnitTests_TestUpdateBehaviour_Test;
#endif
};

#endif  // PSAF_SIGN_DETECTION__SIGN_DETECTION_NODE_HPP_
