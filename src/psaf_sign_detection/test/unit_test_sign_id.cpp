/**
 * @file unit_tests_sign_id.cpp
 * @brief The unit tests for the sign_id namespace
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>

#include "gtest/gtest.h"
#include "psaf_sign_detection/sign_id.hpp"

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

// test get name returns correct results ands detects errors
TEST(SignIdUnitTests, TestGetName) {
  // test function returns the names we define in header
  for (const auto & id : sign_id::signIdNameMap) {
    EXPECT_EQ(
      sign_id::getSignName(id.first),
      sign_id::signIdNameMap.at(id.first)) << "Expect name from function to match the one in map";
  }

  bool gotException = false;

  try {
    // cast invalid underlying value to sign id so we can ub
    sign_id::getSignName(sign_id::underlying2id(-1234));
  } catch (...) {
    gotException = true;
  }

  EXPECT_TRUE(gotException) << "Expect invalid sign id to thrown an exception";
}

// test sign id conversion handles invalid cases
TEST(SignIdUnitTests, TestSignIdToPsafIdInvalid) {
  bool gotException = false;

  try {
    // cast invalid underlying value to sign id so we can ub
    sign_id::signIdToPsafId(sign_id::underlying2id(-1234));
  } catch (...) {
    gotException = true;
  }

  EXPECT_TRUE(gotException) << "Expect invalid sign id to thrown an exception";
}

// test verify model functions works for expected and detects errors
TEST(SignIdUnitTests, TestVerifyModel) {
  auto models_file = getTestDataDir().parent_path().parent_path() / "model" /
    "classes.txt";
  ASSERT_TRUE(std::filesystem::exists(models_file)) <<
    "Expect models.txt to exist";

  // read valid classes from file
  std::vector<std::string> classes;
  std::ifstream infile(models_file.string(), std::ios::in);
  std::string classId;
  while (std::getline(infile, classId)) {
    classes.push_back(classId);
  }

  ASSERT_GT(classes.size(), 0U) << "Expect at least one class to be loaded";

  // check verify model works wiht valid model
  bool gotException = false;
  try {
    sign_id::verifyModel(classes);
  } catch (...) {
    gotException = true;
  }
  EXPECT_FALSE(gotException) << "Expect model to be valid";

  std::vector<std::string> invalidModelToSmall = classes;
  std::vector<std::string> invalidModelNames = classes;

  invalidModelToSmall.resize(1);
  invalidModelNames.at(0) = "TestVerifyModel";

  gotException = false;
  try {
    sign_id::verifyModel(invalidModelToSmall);
  } catch (...) {
    gotException = true;
  }
  EXPECT_TRUE(gotException) << "Expect exception for to small model";

  gotException = false;
  try {
    sign_id::verifyModel(invalidModelNames);
  } catch (...) {
    gotException = true;
  }
  EXPECT_TRUE(gotException) << "Expect exception for model with invalid names";
}
