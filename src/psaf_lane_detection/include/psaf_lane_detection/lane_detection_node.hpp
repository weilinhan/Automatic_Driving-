/**
 * @file lane_detection_node.hpp
 * @brief this class is the lane detection for the car
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
#define PSAF_LANE_DETECTION__LANE_DETECTION_NODE_HPP_

#include <string>
#include <vector>
#include <cstdlib>

#include "opencv4/opencv2/opencv.hpp"
#include "libpsaf/interface/lane_detection_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include "psaf_shared/image_saver.hpp"

/**
 * @brief Struct class used to encapsulate all configs for a ControllerNode
 */
struct LaneDetectionConfig
{
  // update frequency of this node
  double updateFrequency = 60.0;
  // indicate wheter this node should output debug messages or not
  bool debugOutput = false;
  // whether to save generated images, works regardless of debug
  bool saveImages = false;
  // whether to show generated images or not, works regardless of debug
  bool showImages = false;

  // the width of sliding windows
  int slWidth = 60;
  // the height of sliding windows
  int slHeight = 40;

  // the block size of cv::adaptiveThreshold
  unsigned int binarizerblockSize = 125;
  // the constant that is subtracted from the mean or weighted sum of the neighbourhood pixels
  double binarizerC = -58.0;

  // homography parameters
  double homographyData[9] = {
    -0.5749647330671658, -1.5518509466969903, 429.34910456114443,
    -0.020595692589811556, -3.0952198232177457, 717.6198602909957,
    -6.743456757132576e-05, -0.006228181105284407, 1.0};

  // homography used for lane detection
  cv::Mat homography = cv::Mat(3, 3, CV_64F, homographyData);

  /**
   * @brief Load a LaneDetectionConfig from parameters using the provided node
   * @param node the node to declare the parameters on
   * @return LaneDetectionConfig the created instance
   */
  static LaneDetectionConfig fromParameters(rclcpp::Node & node);

  /**
   * @brief Output this config to an ostream
   * @param os the ostream to output to
   * @param cfg the config to output to the stream
   * @return std::ostream& reference to the input ostream
   */
  friend std::ostream & operator<<(std::ostream & os, const LaneDetectionConfig & cfg);
};

/**
 * @class LaneDetectionNode
 * @implements LaneDetectionInterface
 * @brief The Lane detection for the car
 * @details This class is the node implementation of the lane detection.
 * It has 3 tasks:
 * 1. Calculate the position of the lane markings (left, center, right) in the image.
 * 2. Detect the start line (this is only necessary in discipline one of the carolo cup)
 * 3. Detect stop lines (this is only necessary in discipline two of the carolo cup)
 */
class LaneDetectionNode : public libpsaf::LaneDetectionInterface
{
public:
  /**
   * @brief Construct a new Lane Detection Node object
   * @param imageSaver optional pointer to an imagesaver to use
   */
  explicit LaneDetectionNode(psaf_shared::ImageSaver * imageSaver = nullptr);

  /**
   * This method is used to publish the results. It is called periodically by the main loop.
   * Call the publishers in this method.
   */
  void update();

  /**
   * @brief Apply the LaneDetectionConfig to this node
   * @param cfg the LaneDetectionConfig to apply to this node
   */
  void applyConfig(const LaneDetectionConfig & cfg);

  /**
   * @brief Get this nodes LaneDetectionConfig
   * @return LaneDetectionConfig the current LaneDetectionConfig
   */
  LaneDetectionConfig getConfig() const;

protected:
  /**
   * @brief Callback method for the state of the state machine
   * @param[in] state the state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Callback method for the image
   * @param[in] img the camera image
   * @param[in] sensor: the position of the topic name in the topic vector
   * see configuration.hpp for more details
   */
  void processImage(cv::Mat & img, int sensor) final;

  /**
  * Grayscale image
  * @param[in] img a color image
  * @param[out] result the grayscale image
  */
  void grayscaleImage(const cv::Mat & img, cv::Mat & result);

  /**
   * @brief convert the input grayscale image into a binary image
   * @details this method creates a grayscale image into a binary image.
   * @param[in] img the input grayscale image
   * @param[out] result the ouput binary image
   */
  void binarizeImage(const cv::Mat & img, cv::Mat & result);

  /**
  * Transform an image, i.e birdseye view
  * @param[in] img the image to be transformed
  * @param[in] homography the homography matrix
  * @param[out] result the transformed image
  */
  void transformImage(const cv::Mat & img, const cv::Mat & homography, cv::Mat & result);

  /**
   * Resize a given image to 640x480 Pixels
   * @param[in] image the image to be resized
   * @param[out] result the image where the result will be stored in
   */
  void resizeImage(const cv::Mat & image, cv::Mat & result);

  /**
   * @brief Try to find the maximum contour point in the specified image
   * @param image the image to search in
   * @param point output point of maximum contour
   * @return true if the point is valid, else false
   */
  bool getMaxContour(const cv::Mat & image, cv::Point & point) const;

  /**
   * Extract the lane markings from the image
   * The results is a vector of vectors. The inner vectors contain the points of the lane markings
   * They should be left, center, right. If a certain lane is not detected, the vector will be empty
   * The inner vectors are allowed to have different sizes.
   * @param img the image to be processed. This should already be the preprocessed image, i.e binarized
   * @param debugImage optional ptr to image to draw debug info to
   */
  void extractLaneMarkings(const cv::Mat & img, cv::Mat * debugImage = nullptr);

  /**
   * @brief Generate the required debug output of this node
   * @details print statements and images
   * @param the image we searched lane markings on
   * @return true if debug output was generated, else false
   */
  bool generateDebugOutput(const cv::Mat & img) const;

  /**
   * @brief Try to find base points in provided image
   * @param img the image to find base points in
   * @param debugImage optional ptr to image to draw debug info to
   * @return std::vector<cv::Point> result point vector
   */
  std::vector<cv::Point> getBasePoints(const cv::Mat & img, cv::Mat * debugImage = nullptr) const;

  /**
   * @brief Struct used to encapsulate base points and information about them
   */
  struct BasePointInformation
  {
    // x base coordinate of the specific lane
    int lBase = 0, cBase = 0, rBase = 0;
    // whether to search for the specific lane or not
    bool searchL = false, searchC = false, searchR = false;
  };

  /**
   * @brief Generate BasePointInformation for specific points and image
   * @param base_points base points to generate info for
   * @param imageSize size of the image for the base points
   * @return BasePointInformation generated information
   */
  BasePointInformation getBasePointInformation(
    const std::vector<cv::Point> & base_points,
    const cv::Size imageSize) const;

private:
  // Config of this node instance
  LaneDetectionConfig mConfig;
  // ImageSaver instance used to save debug images
  psaf_shared::ImageSaver * const mImageSaver = nullptr;

  // Variable to store the last calculated lane markings
  std::vector<std::vector<cv::Point>> m_lane_markings;

#ifdef BUILD_UNIT_TEST
  friend class LaneDetectionUnitTests;
  friend class LaneDetectionUnitTests_TestExtractLaneMarkingEdgeCases_Test;
  friend class LaneDetectionUnitTests_TestUpdateState_Test;
  friend class LaneDetectionUnitTests_TestDebugOutput_Test;
  friend class LaneDetectionUnitTests_TestUpdatePublishing_Test;
  friend class LaneDetectionUnitTests_TestImageProcessing_Test;
  friend class LaneDetectionUnitTests_TestLaneMarkingExtraction_Test;
#endif
};

#endif  // PSAF_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
