/**
 * @file unit_tests_sign_detector.cpp
 * @brief The unit tests for the sign detector class
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include <filesystem>
#include <algorithm>
#include <iostream>

#include "gtest/gtest.h"
#include "psaf_sign_detection/sign_detector.hpp"

/**
 * @brief Try to get the path of the TEST_DATA_DIR env variable
 * @return std::filesystem::path path if it exists, else an exception is thrown
 */
static std::filesystem::path getTestDataDir()
{
  char * ptr = std::getenv("TEST_DATA_DIR");

  if (ptr == nullptr) {
    throw std::runtime_error(
            "TEST_DATA_DIR environment variable not found,"
            " make sure it is set to the resources folder.");
  }

  std::filesystem::path resources(ptr);

  if (!std::filesystem::exists(resources)) {
    throw std::runtime_error(
            "Path specified by TEST_DATA_DIR does not exist (path=" +
            std::string(resources) + ")");
  }

  return resources;
}

// test sign class constructor works and output doesnt fail
TEST(SignDetectorUnitTests, TestSignClass) {
  // check name and members are one we expect
  for (const auto & id : sign_id::allSignId) {
    SignDetector::Sign testSign(id, 0.43, cv::Rect(1, 2, 3, 4));
    EXPECT_NEAR(testSign.confidence, 0.43, 1e-6) << "Expect confidence to be the one we set it to";
    EXPECT_EQ(
      testSign.boundingBox,
      cv::Rect(1, 2, 3, 4)) << "Expect bounding box to be the one we set it to";
    EXPECT_EQ(
      testSign.name(),
      sign_id::getSignName(id)) << "Expect sign name function to equal names in ::sign_id";
  }

  try {
    std::cout << SignDetector::Sign(sign_id::SignId::Stop, 10, cv::Rect(4, 1, 2, 3)) << std::endl;
  } catch (...) {
    FAIL() << "Expect no exception when operator<< is called for Sign";
  }
}

// test constructor works and throws exceptions for invalid inputs
TEST(SignDetectorUnitTests, TestConstructSignDetector) {
  const auto model_path = getTestDataDir().parent_path().parent_path() / "model";
  const auto model_file = model_path / "classes.txt";
  const auto onnx_file = model_path / "yolov5-7signs.onnx";

  if (!std::filesystem::exists(model_file) && std::filesystem::exists(onnx_file)) {
    throw std::runtime_error("Expect classes.txt and yolov5-7signs.onnx to exists in model/ dir");
  }

  // dont expect failure
  SignDetector succesfull(onnx_file, model_file, 0, 0, 0);

  // read valid classes from file
  std::vector<std::string> classes;
  std::ifstream infile(model_file.string(), std::ios::in);
  std::string classId;
  while (std::getline(infile, classId)) {
    classes.push_back(classId);
  }

  ASSERT_GT(classes.size(), 0U) << "Expect at least one class to be loaded";

  // sort classes so we can compare contents
  auto getNames = succesfull.getClassNames();
  std::sort(getNames.begin(), getNames.end());
  std::sort(classes.begin(), classes.end());
  EXPECT_EQ(getNames, classes) << "Expect SignDetector to contain all classes from file";

  // test error cases
  auto constructSign =
    [&](const std::filesystem::path & onxPath, const std::filesystem::path & modelPath) -> bool {
      bool gotException = false;
      try {
        SignDetector sign(onxPath, modelPath, 0, 0, 0);
      } catch (...) {
        gotException = true;
      }
      return gotException;
    };

  // create files for onx and class loading error testing
  std::filesystem::path nonexistingOnx = std::filesystem::current_path() /
    "TestConstructSignDetectorNonExisting.onnx";
  std::filesystem::path nonexistingClasses = std::filesystem::current_path() /
    "TestConstructSignDetectorNonExisting.txt";
  std::filesystem::path notOnx = std::filesystem::current_path() /
    "TestConstructSignDetector.onnx.not";
  std::filesystem::path notClasses = std::filesystem::current_path() /
    "TestConstructSignDetector.txt.not";
  std::filesystem::path emptyOnx = std::filesystem::current_path() /
    "TestConstructSignDetector.onnx";
  std::filesystem::path emptyClasses = std::filesystem::current_path() /
    "TestConstructSignDetector.txt";

  // create required empty files
  std::vector<std::filesystem::path> toCreate = {notOnx, notClasses, emptyOnx, emptyClasses};
  for (const auto & file : toCreate) {
    std::ofstream create(file);
  }

  EXPECT_TRUE(constructSign(nonexistingOnx, model_file))
    << "Expect an exception with nonexsting onx file";
  EXPECT_TRUE(constructSign(notOnx, model_file)) << "Expect an exception with not onx file";
  EXPECT_TRUE(constructSign(emptyOnx, model_file)) << "Expect an exception with empty onx file";

  EXPECT_TRUE(constructSign(onnx_file, nonexistingClasses))
    << "Expect an exception with nonexsting onx file";
  EXPECT_TRUE(constructSign(onnx_file, notClasses)) << "Expect an exception with not onx file";
  EXPECT_TRUE(constructSign(onnx_file, emptyClasses)) << "Expect an exception with empty onx file";

  // clean up after ourselves
  for (const auto & file : toCreate) {
    if (std::filesystem::exists(file)) {
      std::filesystem::remove(file);
    }
  }
}


// Test Sign Detector detects signs under optimal conditions
TEST(SignDetectorUnitTests, TestDetectsSigns) {
  auto model_path = getTestDataDir().parent_path().parent_path() / "model";
  auto model_file = model_path / "classes.txt";
  auto onnx_file = model_path / "yolov5-7signs.onnx";

  if (!std::filesystem::exists(model_file) && std::filesystem::exists(onnx_file)) {
    throw std::runtime_error("Expect classes.txt and yolov5-7signs.onnx to exists in model/ dir");
  }

  // get sign detector with realistic parameters
  constexpr double testConfidence = 0.6;
  SignDetector detector(onnx_file, model_file, testConfidence, 0.5, 0.4);

  // test empty image
  auto signs = detector.detectSigns(cv::Mat(cv::Size(640, 640), CV_8UC3, cv::Scalar(0, 0, 0)));
  EXPECT_EQ(signs.size(), 0U) << "Expect no signs to be detected in empty image";

  // test empty image with time
  unsigned int elapsedTime = 0x1234;
  signs.clear();
  signs = detector.detectSigns(
    cv::Mat(
      cv::Size(640, 640),
      CV_8UC3, cv::Scalar(255, 255, 255)), &elapsedTime);
  EXPECT_EQ(signs.size(), 0U) << "Expect no signs to be detected in empty image";
  EXPECT_NE(elapsedTime, 0x1234U) << "Expect elapsed time to be see";

  // test with image with all signs from car camera
  signs.clear();

  auto testImage = cv::imread(getTestDataDir() / "TestDetectsSigns7.png");
  ASSERT_FALSE(testImage.empty()) << "Expect test image to not be empty";
  signs = detector.detectSigns(testImage);

  // test we got seven signs our model can detect
  ASSERT_EQ(signs.size(), 7U) << "Expect 7 detected signs";
  // test sign output is sorted
  EXPECT_TRUE(
    std::is_sorted(
      signs.begin(), signs.end(),
      [&](const SignDetector::Sign & first, const SignDetector::Sign & second) {
        return first.confidence >= second.confidence;
      })) << "Expect signs to be sorted by decreasing confidence";

  // test threshold is really applied
  for (const auto & sign : signs) {
    EXPECT_GE(
      sign.confidence,
      testConfidence) << "Expect each sign to have confidence >= threshold";
    EXPECT_GT(sign.boundingBox.area(), 0) << "Expect sign bounding box to be not empty";
  }

  // test we found each possible sign type
  for (const auto & id : sign_id::allSignId) {
    // could also use std::find_if, but this is clearer
    bool found = false;
    for (const auto & sign : signs) {
      if (sign.id == id) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Expect each sign type to be detected";
  }
}
