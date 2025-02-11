/**
 * @file sign_detector.cpp
 * @author sven.kalmbach
 * @date 2024-02-07
 */
#include "psaf_sign_detection/sign_detector.hpp"

SignDetector::SignDetector(
  const std::filesystem::path & yoloOnnxPath,
  const std::filesystem::path & classesPath,
  const double confidenceThresh, const double classThresh, const double nmsThresh)
: mNet(loadNet(yoloOnnxPath)),
  mClassNames(loadClassNames(classesPath)),
  mConfidenceThreshold(confidenceThresh),
  mClassThreshold(classThresh),
  mNmsThreshold(nmsThresh)
{
  // check that sign_id namespace matches loaded classes
  sign_id::verifyModel(mClassNames);
}

SignDetector::Sign::Sign(
  const sign_id::SignId & id, const float confidence,
  const cv::Rect & boundingBox)
: id(id), confidence(confidence), boundingBox(boundingBox)
{
}

std::string SignDetector::Sign::name() const
{
  return sign_id::getSignName(id);
}

std::vector<SignDetector::Sign> SignDetector::detectSigns(cv::Mat image, unsigned int * elapsedMs)
{
  // start time measurement
  auto startTime = std::chrono::system_clock::now();

  // generate and set our model input
  mNet.setInput(getBlob(image));

  // get our results from net
  std::vector<cv::Mat> output;
  mNet.forward(output, mNet.getUnconnectedOutLayersNames());
  auto signs = getDetections(output, image.size());

  // sort signs in descending order
  std::sort(
    signs.begin(), signs.end(),
    [](const auto & lhs, const auto & rhs) {
      return lhs.confidence > rhs.confidence;
    }
  );

  // stop time measurement and output if required
  auto stopTime = std::chrono::system_clock::now();
  if (elapsedMs != nullptr) {
    *elapsedMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime).count();
  }

  return signs;
}

cv::Mat SignDetector::getBlob(const cv::Mat & image) const
{
  // generated input image by resizing to yolov5 input
  cv::Mat resizedImg;
  cv::resize(image, resizedImg, yoloV5InputSize);

  // generate blob we can input into net:
  // scale bgr values to 0.0-1.0, set blob size to mNet input size,
  // no mean, bgr instead of rgb, dont crop
  return cv::dnn::blobFromImage(
    resizedImg, 1.0 / 255.0, yoloV5InputSize,
    cv::Scalar(), true, false);
}

std::vector<SignDetector::Sign> SignDetector::getDetections(
  std::vector<cv::Mat> & netOutput,
  const cv::Size & ogSize)
const
{
  // sanity check for model changes
  if (netOutput.size() != 1) {
    throw std::runtime_error("Unexpected netOutput dimensions!");
  }

  // parsing code is inspired by https://github.com/doleron/yolov5-opencv-cpp-python/blob/main/cpp/yolo.cpp
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<unsigned int> ids;

  // calculate scaling factor for bounding box generation once
  const double widthScale = static_cast<double>(ogSize.width) /
    static_cast<double>(yoloV5InputSize.width);
  const double heightScale = static_cast<double>(ogSize.height) /
    static_cast<double>(yoloV5InputSize.height);

  // pointer to first element of yolo output array
  // we increment pointer instead of indexing, since its
  // clearer with the way yolo outputs data
  // saver way would be a struct for each row
  float * data = reinterpret_cast<float *>(netOutput[0].data);
  // layout of each data row:
  // (x, y, width, height, confidence, classes_score[0], ..., classes_score[n-1])

  for (unsigned int i = 0; i < mYolov5DRows; ++i) {
    // filter for confidence of detection
    float confidence = data[4];

    if (confidence > mConfidenceThreshold) {
      // find class with max score and check if max score is over our threshold
      float * class_scores = data + 5;

      // use opencv to find class with max score
      double maxClassScore;
      cv::Point classId;
      // create 1d array from class scores
      cv::Mat scores(1, mClassNames.size(), CV_32FC1, class_scores);
      // we only care about max point
      cv::minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &classId);

      // if max class is over threshold, we got a detection
      if (maxClassScore > mClassThreshold) {
        // construct inputs for our nms filter
        // convert yolo output to cv rect with scale taken into consideration
        float x = data[0];
        float y = data[1];
        float width = data[2];
        float height = data[3];

        x = (x - 0.5 * width) * widthScale;
        y = (y - 0.5 * height) * heightScale;
        width *= widthScale;
        height *= heightScale;

        boxes.push_back(cv::Rect(x, y, width, height));
        confidences.push_back(confidence);
        ids.push_back(classId.x);
      }
    }

    // skip to next row
    data += mRowEntries;
  }

  // yolo outputs multiple boxes for single detection, use nms filtering to get rid of them
  std::vector<int> nmsResults;
  cv::dnn::NMSBoxes(boxes, confidences, mClassThreshold, mNmsThreshold, nmsResults);

  // generate sign objects from results
  std::vector<Sign> signResults;
  for (const auto & resIndex : nmsResults) {
    auto sign =
      Sign(sign_id::underlying2id(ids.at(resIndex)), confidences.at(resIndex), boxes.at(resIndex));
    signResults.push_back(sign);
  }

  return signResults;
}

cv::dnn::Net SignDetector::loadNet(const std::filesystem::path & path) const
{
  // check file exists
  if (std::filesystem::exists(path)) {
    // check extension matches expected
    if (path.extension() == ".onnx") {
      auto net = cv::dnn::readNetFromONNX(path.string());
      // check net is not empty
      if (!net.empty()) {
        return net;
      } else {
        throw std::runtime_error(
                "Failed to load net: net in provided file \"" + path.string() + "\" is empty!");
      }
    } else {
      throw std::runtime_error(
              "Failed to load net: provided file \"" + path.string() + "\" does not end in .onnx!");
    }
  } else {
    throw std::runtime_error(
            "Failed to load net: provided file \"" + path.string() + "\" does not exist!");
  }
}

std::vector<std::string> SignDetector::getClassNames() const
{
  return mClassNames;
}


std::vector<std::string> SignDetector::loadClassNames(const std::filesystem::path & path) const
{
  // check file exists
  std::vector<std::string> classes;

  if (std::filesystem::exists(path)) {
    // check extension matches expected
    if (path.extension() == ".txt") {
      // open file in read mode
      std::ifstream infile(path.string(), std::ios::in);

      // read class ids line by line
      std::string classId;
      while (std::getline(infile, classId)) {
        classes.push_back(classId);
      }

      // check we actually read something
      if (classes.size() == 0) {
        throw std::runtime_error(
                "Failed to load classes: read result of \"" + path.string() +
                "\" is empty .txt!");
      }

      return classes;

    } else {
      throw std::runtime_error(
              "Failed to load classes: provided file \"" + path.string() +
              "\" does not end in .txt!");
    }
  } else {
    throw std::runtime_error(
            "Failed to load classes: provided file \"" + path.string() + "\" does not exist!");
  }
}

std::ostream & operator<<(std::ostream & os, const SignDetector::Sign & sign)
{
  os << "Sign(id=" << sign_id::id2underlying(sign.id) << ", name=" <<
    sign.name() << ", confidence=" << sign.confidence << ", box=" <<
    sign.boundingBox << ")";
  return os;
}
