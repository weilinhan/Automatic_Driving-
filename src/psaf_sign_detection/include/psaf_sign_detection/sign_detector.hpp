/**
 * @file sign_detector.hpp
 * @brief The definition of the SignDetector class
 * @author sven.kalmbach
 * @date 2024-02-07
 */
#ifndef PSAF_SIGN_DETECTION__SIGNDETECTOR_HPP_
#define PSAF_SIGN_DETECTION__SIGNDETECTOR_HPP_

#include <string>
#include <filesystem>
#include <vector>
#include <fstream>

#include "opencv4/opencv2/opencv.hpp"
#include "psaf_sign_detection/sign_id.hpp"

/**
 * @class SignDetectionNode
 * @brief Detect signs in provided 640x480 images
 */
class SignDetector
{
public:
  /**
   * @class Sign
   * @brief Class/Struct to encapsulate sign info
   */
  class Sign
  {
public:
    /**
     * @brief Construct a new Sign
     * @param id id of the sign
     * @param confidence confidence of its detection
     * @param boundingBox bounding box around the sign in the frame it was detected
     */
    Sign(const sign_id::SignId & id, const float confidence, const cv::Rect & boundingBox);

    /**
     * @brief Get the name of this sign
     * @return std::string the name of this sign
     */
    std::string name() const;

    // struct variables
    sign_id::SignId id;
    float confidence;
    cv::Rect boundingBox;

    // overloaded stream operator for clean output
    friend std::ostream & operator<<(std::ostream & os, const Sign & sign);
  };

  /**
  * @brief Construct a new SignDetector
  * @param yoloOnnxPath path to the yolo v5 .onnx file
  * @param classesPath path to the classes .txt file
  * @param confidenceThresh threshold for network confidences
  * @param classThresh threshold for class confidences
  * @param nmsThresh threshold for nms box filtering
  */
  SignDetector(
    const std::filesystem::path & yoloOnnxPath,
    const std::filesystem::path & classesPath,
    const double confidenceThresh, const double classThresh, const double nmsThresh);

  /**
   * @brief Try to find all signs in an image
   * @param image the image to search signs for
   * @param elapsedMs optional output pointer to processing time in ms
   * @return std::vector<Sign> Vector containing all detected signs, sorted in order of decreasing confidence
   */
  std::vector<Sign> detectSigns(cv::Mat image, unsigned int * elapsedMs = nullptr);

  /**
   * @brief Get the class name vector loaded from input file
   * @details class_id corresponds to vector index of name
   * @return std::vector<std::string> the class names
   */
  std::vector<std::string> getClassNames() const;

protected:
  /**
   * @brief Try to load the yolo .onnx from the provided path and sanity check the net
   * @details Throws an exception if model could not be loaded
   * @param path path to the .onnx file to load the net from
   * @return cv::dnn::Net the loaded net
   */
  cv::dnn::Net loadNet(const std::filesystem::path & path) const;

  /**
   * @brief Try to load yolo classes from the provided .txt file path
   * @details - first entry is assumed to be id 0, second entry id 1, ...
   *          - text file should only contain names and newlines, nothing else
   *          - throws an exception if loading failed
   * @param path path to the .txt file to load the classes from
   * @return std::vector<std::string> the vector containg class files
   */
  std::vector<std::string> loadClassNames(const std::filesystem::path & path) const;

  /**
   * @brief Get a correctly sized cv::Mat blob from a camera input image
   * @details Throws an exception if assumptions dont match
   * @param image the input image, expected to be in BGR
   * @return cv::Mat the output blob, ready for yolov5 (rgb, 640x640, 1/255 scale)
   */
  cv::Mat getBlob(const cv::Mat & image) const;

  /**
   * @brief Get Sign detections from the provided network output
   * @details Applies confidence, class and nms thresholding to network output
   * @param netOutput the yolo v5 network output to process
   * @param ogSize size of the input image, used for scaling
   * @return std::vector<Sign> vector containg all detections over thresholds
   */
  std::vector<Sign> getDetections(std::vector<cv::Mat> & netOutput, const cv::Size & ogSize) const;

private:
  // the dnn we use to process images
  cv::dnn::Net mNet;
  // vector containg class names, index is their id
  const std::vector<std::string> mClassNames;

  // limit confidence score of detection, if lower than this detection is not considered
  const double mConfidenceThreshold;
  // limit class threshold, only if highest class threshold is higher/equal
  const double mClassThreshold;
  // threshold for nms box filtering
  const double mNmsThreshold;

  // specific number of rows in yolov5 output, used to iterate over output
  static constexpr unsigned int mYolov5DRows = 25200;
  // 5 fixed params (x,y, height, width, confidence) and n class confidence scores in each column
  const unsigned int mRowEntries = 5 + mClassNames.size();
  // input size used by yolov5
  const cv::Size yoloV5InputSize = cv::Size(640, 640);
};

#endif  // PSAF_SIGN_DETECTION__SIGNDETECTOR_HPP_
