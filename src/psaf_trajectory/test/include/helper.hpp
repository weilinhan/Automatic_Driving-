/**
 * @file helper.cpp
 * @brief Helpers for trajectory node unit testing
 * @author PSAF
 * @date 2023-12-05
 */
#ifndef PSAF_TRAJECTORY_HELPER_HPP
#define PSAF_TRAJECTORY_HELPER_HPP

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "libpsaf_msgs/msg/lane_markings.hpp"

// use namespace to keep global namespace clean
namespace trajectory_testhelper
{

// constants for parsing .lanes and .traj files
static constexpr const char * LEFT_LANE_PREAMBLE = "left_lane:";
static constexpr const char * CENTER_LANE_PREABMLE = "center_lane:";
static constexpr const char * RIGHT_LANE_PREAMBLE = "right_lane:";
static constexpr const char * TRAJECTORY_PREAMBLE = "trajectory:";
static constexpr const char DELIMITER = ',';

// constants for file endings
static constexpr const char * LANEMARKINGS_EXTENSION = ".lanes";
static constexpr const char * TRAJECTORY_EXTENSION = ".traj";
static constexpr const char * OUT_EXTENSION = ".out";

/**
 * @brief Container class for a pair of input lanes and an output trajectory
 */
class TrajectoryTestPair
{
public:
/**
 * @brief Construct a new Trajectory Test Pair object
 *
 * @param trajectory path to the .traj file containing the expected trajectory
 * @param lanes path to the .lanes file containg the input lane markings
 */
  TrajectoryTestPair(const std::filesystem::path & trajectory, const std::filesystem::path & lanes)
  : trajectory(trajectory), lanes(lanes)
  {
    if (!exists()) {
      throw std::runtime_error(
              "Tried to construct TrajectoryTestPair"
              " from non existent file paths!");
    }
  }

  /**
   * @brief Check if both files contained by this class exist
   * @return true if both files exists, else false
   */
  bool exists() const
  {
    return std::filesystem::exists(trajectory) && std::filesystem::exists(lanes);
  }

  // actual members to store paths
  std::filesystem::path trajectory, lanes;
};

/**
 * @brief Enum used for .lanes parsing, indicates current state of parser
 */
enum class LanesParsingState
{
  left_lane,
  center_lane,
  right_lane,
  searching
};

/**
  * @brief Split a string into multiple substring
  * Code copied from https://stackoverflow.com/a/46931770
  * @param s string which to split
  * @param delim delimiter at which to split strings at
  * @return std::vector<std::string> a vector containg the substrings
  */
std::vector<std::string> split(const std::string & s, const char delim)
{
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;

  while (getline(ss, item, delim)) {
    result.push_back(item);
  }

  return result;
}

/**
 * @brief Parse LaneMarkings from a file
 * @param file file to parse the markings from
 * @return libpsaf_msgs::msg::LaneMarkings object containing the parsed lane points
 */
libpsaf_msgs::msg::LaneMarkings parseLaneMarkingsFromFile(const std::filesystem::path & file)
{
  if (!std::filesystem::exists(file)) {
    std::cerr << "Requested file doesnt exist, exitting..." <<
      std::endl;
    std::exit(-1);
  }

  if (file.extension() != LANEMARKINGS_EXTENSION) {
    throw std::runtime_error(
            "Specified file extension doesnt match"
            " LANEMARKINGS_EXTENSION (extension=" + std::string(
              file.extension()) + ")");
  }

  auto markings = libpsaf_msgs::msg::LaneMarkings();
  std::ifstream in;
  LanesParsingState mode = LanesParsingState::searching;

  in.open(file, std::ios::in);

  if (!in.is_open()) {
    throw std::runtime_error(
            "Failed to open file to parse lane markings from (file=" +
            std::string(file) + ")");
  }

  std::string line;

  // while we still can read lines
  while (getline(in, line)) {
    // ignore empty lines
    if (line.size() == 0) {
      continue;
    }

    // look for preamble, if we find one, we change parsing mode
    // and skip current line. If we dont find one we keep searching
    if (line == LEFT_LANE_PREAMBLE) {
      mode = LanesParsingState::left_lane;
      continue;
    } else if (line == CENTER_LANE_PREABMLE) {
      mode = LanesParsingState::center_lane;
      continue;
    } else if (line == RIGHT_LANE_PREAMBLE) {
      mode = LanesParsingState::right_lane;
      continue;
    }

    // if we arent searching for a line header, we can start parsing points
    if (mode != LanesParsingState::searching) {
      geometry_msgs::msg::Point p;

      // split line into 3 substrings divided by DELIMITER (,)
      std::vector<std::string> vals = split(line, DELIMITER);

      if (vals.size() != 3) {
        throw std::runtime_error(
                "Failed to parse 3 lane coordinates from line, check formatting (line=" + line +
                ")");
      }

      // convert strings to point coordinates
      p.x = std::stod(vals.at(0));
      p.y = std::stod(vals.at(1));
      p.z = std::stod(vals.at(2));

      // push point to correct lane
      switch (mode) {
        case LanesParsingState::left_lane:
          markings.left_lane.push_back(p);
          break;

        case LanesParsingState::right_lane:
          markings.right_lane.push_back(p);
          break;

        case LanesParsingState::center_lane:
          markings.center_lane.push_back(p);
          break;

        default:
          throw std::runtime_error("Invalid parsing mode enum value encountered!");
      }
    }
  }

  in.close();
  return markings;
}

/**
 * @brief Print the points of a trajector to an outstream
 * @param trajectory trajector to print
 * @param out stream to print to, std::cout by default
 */
void printTrajectory(
  const libpsaf_msgs::msg::Trajectory & trajectory,
  std::ostream & out = std::cout)
{
  out << "== Printing Trajectory ==\n";

  for (auto & point : trajectory.points) {
    out << point.x << ", " << point.y << ", " << point.z << "\n";
  }

  out << "======== Done ===========\n" << std::flush;
}

/**
 * @brief Write a trajectory to filePath, in a parseable format
 * @param trajectory the trajectory to write out
 * @param filePath path of the file to write to, will be overwritten if it already exists
 */
void writeTrajectoryToFile(
  const libpsaf_msgs::msg::Trajectory & trajectory,
  const std::filesystem::path & file)
{
  std::ofstream out;

  // try to open file, exception if failed
  out.open(file, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    throw std::runtime_error(
            "Could not open file to write trajectory (file=" + std::string(
              file) + ")");
  }

  // write points with preamble
  out << TRAJECTORY_PREAMBLE << "\n";
  for (auto & point : trajectory.points) {
    out << point.x << DELIMITER << " " << point.y << DELIMITER << " " << point.z << "\n";
  }
  out.close();
}

/**
 * @brief Read a trajectory from a file
 * @param file file to read a trajectory from
 * @return libpsaf_msgs::msg::Trajectory parsed trajectory
 */
libpsaf_msgs::msg::Trajectory parseTrajectoryFromFile(const std::filesystem::path & file)
{
  auto traj = libpsaf_msgs::msg::Trajectory();
  bool first = true;

  if (!std::filesystem::exists(file)) {
    throw std::runtime_error(
            "Specified file to parse trajectory from doesnt exist (file=" + std::string(
              file) + ")");
  }

  if (file.extension() != TRAJECTORY_EXTENSION) {
    throw std::runtime_error(
            "Specified file extension doesnt match TRAJECTORY_EXTENSION (extension=" + std::string(
              file.extension()) + ")");
  }

  std::ifstream in;
  in.open(file, std::ios::in);

  if (!in.is_open()) {
    throw std::runtime_error(
            "Failed to open file to parse trajectory from (file=" +
            std::string(file) + ")");
  }

  std::string line;

  // while we still can read lines
  while (getline(in, line)) {
    // ignore empty lines
    if (line.size() == 0) {
      continue;
    }

    // check first non empty line is traj_preamble
    if (first) {
      first = false;
      if (line == TRAJECTORY_PREAMBLE) {
        // skip this line for parsing
        continue;
      } else {
        throw std::runtime_error(
                "First non-empty line of trajectory file doesnt match TRAJECTORY_PREAMBLE (file=" +
                std::string(file) + ")");
      }
    }

    geometry_msgs::msg::Point p;
    // split line into 3 substrings divided by DELIMITER (,)
    std::vector<std::string> vals = split(line, DELIMITER);

    if (vals.size() != 3) {
      throw std::runtime_error(
              "Failed to parse 3 trajectory coordinates from line, check formatting (line=" + line +
              ")");
    }

    // convert strings to point coordinates and push back point
    p.x = std::stod(vals.at(0));
    p.y = std::stod(vals.at(1));
    p.z = std::stod(vals.at(2));
    traj.points.push_back(p);
  }

  in.close();
  return traj;
}

/**
 * @brief Return a vector of TrajectoryTestPair found in baseDir with matching baseName
 * @param baseDir directory to search for .traj and .lanes files
 * @param baseName base filename of .traj and .lane files to match
 * @return std::vector<TrajectoryTestPair> vector with test pairs, empty if none found
 */
std::vector<TrajectoryTestPair> getTrajectoryTestFiles(
  const std::filesystem::path & baseDir,
  const std::string & baseName)
{
  // vector contains resulting lane and traj pairs
  std::vector<TrajectoryTestPair> files;

  if (!std::filesystem::exists(baseDir)) {
    throw std::runtime_error(
            "Requested directory does not exist (baseDir=" + std::string(
              baseDir) + ")");
  }

  // iterate over all files in baseDir
  // NOT recursive over subdirs!
  for (auto & p : std::filesystem::directory_iterator(baseDir)) {
    auto path = p.path();
    // check that file name contains our specified baseName, skip if it doesnt
    if (std::string(path.stem()).find(baseName) == std::string::npos) {
      continue;
    }

    // check for test input .lanes first
    if (path.extension() == LANEMARKINGS_EXTENSION) {
      // construct counterpart .traj path
      std::filesystem::path counterPart = path.parent_path() /
        (std::string(path.stem()) + TRAJECTORY_EXTENSION);

      // check if counterpart exists
      if (!std::filesystem::exists(counterPart)) {
        throw std::runtime_error(
                "Can't find matching .traj file for .lane file (.lane=" + std::string(
                  path) + ", .traj=" + std::string(counterPart) + ")");
      }

      // store our pair
      files.push_back(TrajectoryTestPair(counterPart, path));
    }
  }

  return files;
}

/**
 * @brief Try to parse the resources directory from the TEST_DATA_DIR environment variable.
 * An Exception will be thrown if TEST_DATA_DIR is not set or does not exist.
 * @return std::filesystem::path path of the resources/ dir, if it exists
 */
std::filesystem::path getTestDataDir()
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

void compareTrajectories(
  const libpsaf_msgs::msg::Trajectory & t1,
  const libpsaf_msgs::msg::Trajectory & t2,
  const double abs)
{
  // assert since further comparison doesnt make sense if points are off
  ASSERT_EQ(
    t1.points.size(),
    t2.points.size()) << "Expected both trajectories to have the same number of points!";

  for (size_t i = 0; i < t1.points.size(); ++i) {
    // use gtest for float comparision
    ASSERT_NEAR(
      t1.points.at(i).x, t2.points.at(i).x,
      abs) << "Expected trajectory point x coordinates to be (near) equal!";
    ASSERT_NEAR(
      t1.points.at(i).y, t2.points.at(i).y,
      abs) << "Expected trajectory point y coordinates to be (near) equal!";
    ASSERT_NEAR(
      t1.points.at(i).z, t2.points.at(i).z,
      abs) << "Expected trajectory point z coordinates to be (near) equal!";
  }
}

}  // namespace trajectory_testhelper
#endif  // PSAF_TRAJECTORY_HELPER_HPP
