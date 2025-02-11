/**
 * @file unit_tests.cpp
 * @brief The unit tests main file the psaf shared code
 * @author sven kalmbach
 * @date 2024-02-22
 */
#include "gtest/gtest.h"

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
