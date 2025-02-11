/**
 * @file unit_tests.cpp
 * @brief The unit tests for the psaf shared ImageSaver class
 * @author sven.kalmbach
 * @date 2024-02-22
 */
#include <filesystem>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>

#include "gtest/gtest.h"
#include "psaf_shared/image_saver.hpp"
#include "opencv4/opencv2/opencv.hpp"

using psaf_shared::ImageSaver;

// test imagsaver creates and returns expected output folder if we use cwd
TEST(ImageSaverUnitTests, TestCreateImageSaverInitCWD)
{
  std::string outputFolderName = "TestCreateImageSaverInitCWD";
  std::string parentFolderName = "images";
  std::filesystem::path outputFolderPath = std::filesystem::current_path() / parentFolderName /
    outputFolderName;

  // delete expected output folder if it exists
  if (std::filesystem::exists(outputFolderPath)) {
    std::filesystem::remove_all(outputFolderPath);
  }

  // check that the folder doesnt exists
  ASSERT_FALSE(std::filesystem::exists(outputFolderPath)) <<
    "Expect ImageSaver outputFolder to not exists.";

  // create our instance in cwd
  ImageSaver saver(outputFolderName);

  // check that the folder was created
  ASSERT_TRUE(std::filesystem::exists(outputFolderPath)) <<
    "Expect ImageSaver outputFolder to exists.";

  ASSERT_EQ(
    outputFolderPath.parent_path().filename(),
    parentFolderName)
    << "Expect parent folder of output to be images";

  // make sure the getOutputPath functions works
  // need to compare against parent path as we expect timestamped subfolder
  EXPECT_EQ(saver.getOutputPath().parent_path().compare(outputFolderPath), 0) <<
    "Expect result of getOutputPath to be our folder";

  // clean up after ourselves
  std::filesystem::remove_all(outputFolderPath);
}

// test imagsaver creates and returns expected output folder if we dont use cwd
TEST(ImageSaverUnitTests, TestCreateImageSaverInit)
{
  std::string outputFolderName = "TestCreateImageSaverInit";
  std::string baseFolderName = "TestCreateImageSaverInitFolder";

  std::filesystem::path baseFolderPath = std::filesystem::current_path() / baseFolderName;
  std::filesystem::path outputFolderPath = baseFolderPath / outputFolderName;

  // delete base folder if it exists
  if (std::filesystem::exists(baseFolderPath)) {
    std::filesystem::remove_all(baseFolderPath);
  }

  // check that the base folder doesnt exists
  ASSERT_FALSE(std::filesystem::exists(baseFolderPath)) <<
    "Expect ImageSaver baseFolder to not exists.";

  // create our instance in baseFolder
  ImageSaver saver(outputFolderName, baseFolderPath);

  // check that the folder was created
  ASSERT_TRUE(std::filesystem::exists(outputFolderPath)) <<
    "Expect ImageSaver outputFolder to exists.";

  // make sure the getOutputPath functions works
  EXPECT_EQ(saver.getOutputPath().parent_path().compare(outputFolderPath), 0) <<
    "Expect result of getOutputPath to be our folder";

  // clean up after ourselves
  std::filesystem::remove_all(baseFolderPath);
}

// check that saving with the default parameters behaves as expected
TEST(ImageSaverUnitTests, TestSaveDefault)
{
  std::filesystem::path path = std::filesystem::current_path() / "TestBasicSave";

  // delete expected output folder if it exists
  if (std::filesystem::exists(path)) {
    std::filesystem::remove_all(path);
  }

  // check that the folder doesnt exists
  ASSERT_FALSE(std::filesystem::exists(path)) << "Expect ImageSaver outputFolder to not exists.";

  // create our instance in cwd
  ImageSaver saver(path);

  // check that the folder was created
  ASSERT_TRUE(std::filesystem::exists(path)) << "Expect ImageSaver outputFolder to exists.";

  // create green image and save it
  cv::Mat image(cv::Size(640, 640), CV_8UC3, cv::Scalar(0, 255, 0));
  EXPECT_TRUE(saver.save(image)) << "Expect image to be saved.";
  // sleep for 5 ms so we dont get an error if we create image with the same timestamp!
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_FALSE(saver.save(image)) << "Expect identical image not to be saved again.";

  // find single timestamped output folder
  std::vector<std::filesystem::path> entries;
  for (const auto & entry : std::filesystem::directory_iterator(path)) {
    entries.push_back(entry);
  }
  ASSERT_EQ(entries.size(), 1U) << "Expect a single timestamp subfolder.";

  // find "default" output folder
  std::vector<std::filesystem::path> subEntries;
  for (const auto & entry : std::filesystem::directory_iterator(entries.at(0))) {
    subEntries.push_back(entry);
  }
  ASSERT_EQ(subEntries.size(), 1U) << "Expect a single subFolder.";
  ASSERT_TRUE(
    subEntries.at(0).filename() ==
    "default")
    << "Expected subFolder name to be 'default'";

  // find the single output file
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::directory_iterator(subEntries.at(0))) {
    files.push_back(entry);
  }
  ASSERT_EQ(files.size(), 1U) << "Expect a single file";

  // read file and compare it against the one we saved
  auto readImage = cv::imread(files.at(0));
  EXPECT_TRUE(
    sum(image != readImage) ==
    cv::Scalar(0, 0, 0, 0))
    << "Expect image to be the one we saved.";

  // clean up after ourselves
  std::filesystem::remove_all(path);
}

// check that we can save repeat images if the flag is set
TEST(ImageSaverUnitTests, TestSaveRepeatMultiple)
{
  std::filesystem::path path = std::filesystem::current_path() / "TestBasicSave";

  // delete expected output folder if it exists
  if (std::filesystem::exists(path)) {
    std::filesystem::remove_all(path);
  }

  // check that the folder doesnt exists
  ASSERT_FALSE(std::filesystem::exists(path)) << "Expect ImageSaver outputFolder to not exists.";

  // create our instance in cwd
  ImageSaver saver(path);

  // check that the folder was created
  ASSERT_TRUE(std::filesystem::exists(path)) << "Expect ImageSaver outputFolder to exists.";

  // create red output image three times
  cv::Mat image(cv::Size(125, 125), CV_8UC3, cv::Scalar(0, 0, 255));
  EXPECT_TRUE(saver.save(image, "default", false)) << "Expect image to be saved.";
  // sleep for 5 ms so we dont get an error if we create image with the same timestamp!
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(saver.save(image, "default", false)) << "Expect identical image to be saved again.";
  // sleep for 5 ms so we dont get an error if we create image with the same timestamp!
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(saver.save(image, "default", false)) << "Expect identical image to be saved again.";

  // find single timestamped output folder
  std::vector<std::filesystem::path> entries;
  for (const auto & entry : std::filesystem::directory_iterator(path)) {
    entries.push_back(entry);
  }
  ASSERT_EQ(entries.size(), 1U) << "Expect a single timestamp subfolder.";

  // find "default" output folder
  std::vector<std::filesystem::path> subEntries;
  for (const auto & entry : std::filesystem::directory_iterator(entries.at(0))) {
    subEntries.push_back(entry);
  }
  ASSERT_EQ(subEntries.size(), 1U) << "Expect a single subFolder.";
  ASSERT_TRUE(
    subEntries.at(0).filename() ==
    "default")
    << "Expected subFolder name to be 'default'";

  // find the three output file
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::directory_iterator(subEntries.at(0))) {
    files.push_back(entry);
  }
  ASSERT_EQ(files.size(), 3U) << "Expect three files";

  // read files and compare it against the one we saved
  for (const auto & file : files) {
    auto readImage = cv::imread(file);
    EXPECT_TRUE(
      sum(image != readImage) ==
      cv::Scalar(0, 0, 0, 0))
      << "Expect image to be the one we saved.";
  }

  // clean up after ourselves
  std::filesystem::remove_all(path);
}

// check that saving for multiple image classes works
TEST(ImageSaverUnitTests, TestSaveMultiple)
{
  std::filesystem::path path = std::filesystem::current_path() / "TestSaveMultiple";

  // delete expected output folder if it exists
  if (std::filesystem::exists(path)) {
    std::filesystem::remove_all(path);
  }

  // check that the folder doesnt exists
  ASSERT_FALSE(std::filesystem::exists(path)) << "Expect ImageSaver outputFolder to not exists.";

  // create our instance in cwd
  ImageSaver saver(path);

  // check that the folder was created
  ASSERT_TRUE(std::filesystem::exists(path)) << "Expect ImageSaver outputFolder to exists.";

  // create green image and save it
  cv::Mat green(cv::Size(640, 640), CV_8UC3, cv::Scalar(0, 255, 0));
  cv::Mat red(cv::Size(640, 640), CV_8UC3, cv::Scalar(0, 0, 255));
  EXPECT_TRUE(saver.save(green, "green")) << "Expect green image to be saved.";
  // sleep for 5 ms so we dont get an error if we create image with the same timestamp!
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(saver.save(red, "red")) << "Expected red image to be saved";
  // sleep for 5 ms so we dont get an error if we create image with the same timestamp!
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_FALSE(saver.save(green, "green")) << "Expect green image not to be saved again.";
  // sleep for 5 ms so we dont get an error if we create image with the same timestamp!
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_FALSE(saver.save(red, "red")) << "Expected red image not to be saved again";

  // find single timestamped output folder
  std::vector<std::filesystem::path> entries;
  for (const auto & entry : std::filesystem::directory_iterator(path)) {
    entries.push_back(entry);
  }
  ASSERT_EQ(entries.size(), 1U) << "Expect a single timestamp subfolder.";

  // find green and red output folder
  std::vector<std::filesystem::path> subEntries;
  for (const auto & entry : std::filesystem::directory_iterator(entries.at(0))) {
    subEntries.push_back(entry);
  }
  ASSERT_EQ(subEntries.size(), 2U) << "Expect two subFolders.";

  // find the output files for each folder and compare them
  for (const auto & dir : subEntries) {
    std::vector<std::filesystem::path> files;
    const bool isGreen = dir.filename() == "green";
    const bool isRed = dir.filename() == "red";

    for (const auto & entry : std::filesystem::directory_iterator(dir)) {
      files.push_back(entry);
    }
    ASSERT_EQ(files.size(), 1U) << "Expect a single file";

    // read file and compare it against the one we saved
    auto readImage = cv::imread(files.at(0));

    if (isGreen) {
      EXPECT_TRUE(
        sum(green != readImage) ==
        cv::Scalar(0, 0, 0, 0))
        << "Expect image to be the one we saved.";
    } else if (isRed) {
      EXPECT_TRUE(
        sum(red != readImage) ==
        cv::Scalar(0, 0, 0, 0))
        << "Expect image to be the one we saved.";
    } else {
      FAIL() << "Expected red or green folder.";
    }
  }

  // clean up after ourselves
  std::filesystem::remove_all(path);
}
