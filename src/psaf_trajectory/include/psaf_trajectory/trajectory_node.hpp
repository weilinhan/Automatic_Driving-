/**
 * @file trajectory_node.hpp
 * @brief Definition of the trajectory node
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_TRAJECTORY__TRAJECTORY_NODE_HPP_
#define PSAF_TRAJECTORY__TRAJECTORY_NODE_HPP_

#include <string>
#include <vector>
#include <cmath>

#include "opencv4/opencv2/opencv.hpp"
#include "libpsaf/interface/trajectory_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "lane_kalman_filter.hpp"
#include "psaf_shared/image_saver.hpp"

/**
 * @brief Struct class used to encapsulate all configs for a TrajectoryNode
 */
struct TrajectoryConfig
{
  // this nodes update frequency
  // NOTE: only changed on inital load
  double updateFrequency = 60.0;
  // wheter to enable debug logging or not
  bool debugOutput = false;
  // enable realtime images of generated trajectory
  bool showImages = false;
  // whether to generate trajectories regardless of car state
  bool ignoreState = false;
  // whether to save generated images to files, will also work if mShowImages is set to false
  bool saveImages = false;

  // offsets applied to reference lanes during calculation
  double leftLaneOffset = -5;
  double centerLaneOffset = -2.0;
  double rightLaneOffset = 2.0;

  // min point count for reference lane
  unsigned int minPointCount = 2;
  // min vector offset calculation result length
  double minVecLength = 1e-6;

  // whether to enable kalman filtering of of lanes or not
  bool enableKalman = false;

  // data matrixces for cv matrices, read in from paramters
  double rotMatData[4] = {1.0, 0.0, 0.0, 1.0};
  double transMatData[2] = {0.0, 0.0};

  // matrices to move vectors translational from picture to vehicle coordinate system
  cv::Mat rotMat = cv::Mat(2, 2, CV_64F, rotMatData);
  cv::Mat transMat = cv::Mat(2, 1, CV_64F, transMatData);

  float kProcessNoiseCoeff = 1e-6;
  float kMaxErrorPointDistance = 1e-6;
  float kMeasureNoiseCov[2] = {1e-6, 1e-6};
  float kStateStart[4] = {1e-6, 1e-6, 1e-6, 1e-6};

  /**
   * @brief Load a TrajectoryConfig from parameters using the provided node
   * @param node the node to declare the parameters on
   * @return TrajectoryConfig the created instance
   */
  static TrajectoryConfig fromParameters(rclcpp::Node & node);

  /**
   * @brief Output this config to an ostream
   * @param os the ostream to output to
   * @param cfg the config to output to the stream
   * @return std::ostream& reference to the input ostream
   */
  friend std::ostream & operator<<(std::ostream & os, const TrajectoryConfig & cfg);
};

/**
 * @class TrajectoryNode
 * @implements TrajectoryInterface
 * @brief Calculate the trajectory based on the detected lane markings and obstacles
 * @details This class is the main class of the trajectory node. It is responsible for
 * the calculation of the trajectory and the publishing of the trajectory.
 * The trajectory is being calculated from the detected lanemarkings under consideration of obstacles.
 */
class TrajectoryNode : public libpsaf::TrajectoryInterface
{
public:
  /**
   * @brief Construct a new Trajectory Node object
   * @param imageSaver optional pointer to a imagesaver to use
   */
  explicit TrajectoryNode(psaf_shared::ImageSaver * imageSaver = nullptr);

  /**
   * @brief Destroy the Trajectory Node object
   */
  ~TrajectoryNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

  /**
   * @brief Apply the TrajectoryConfig to this node
   * @param cfg the TrajectoryConfig to apply to this node
   */
  void applyConfig(const TrajectoryConfig & cfg);

  /**
   * @brief Get the current config of this node
   * @return TrajectoryConfig the current node config
   */
  TrajectoryConfig getConfig() const;

protected:
  /**
   * @brief enum used to differentitate lane type when passed as parameter
   */
  enum class LaneType
  {
    LEFT_LANE = 0,
    CENTER_LANE = 1,
    RIGHT_LANE = 2
  };

  // shorthand type for vector of geometry points
  using PointVector = std::vector<geometry_msgs::msg::Point>;

  /**
   * The callback Method for the state
   * @param[in] state the current state of the StateMachine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * The callback Method for the detected object
   * @param[in] p the detected object
   */
  void processObstacle(libpsaf_msgs::msg::Obstacle::SharedPtr p) override;

  /**
   * @brief The callback Method for the detected lane markings
   * @param[in] p the detected lane markings
   */
  void processLaneMarkings(libpsaf_msgs::msg::LaneMarkings::SharedPtr p) override;

  /**
   * @brief Generate a Trajectory from the provided lane markings
   * @param lane vector containing lane marking points
   * @param laneType type of the lane vector, e.g. which lane it represents
   * @return PointVector vector containg the calculated trajectory points
   */
  PointVector offsetLaneToTrajectory(const PointVector & lane, const LaneType laneType) const;

  /**
   * @brief Choose the lane with the most points and return its type
   * @param leftLane the lane to be considered as left lane
   * @param centerLane the lane to be considered as right lane
   * @param rightLane the lane to be considered as center lane
   * @return LaneType indicating the lane containing the most points, if lanes are equal
   *         right lane will be prefered
   */
  LaneType chooseReferenceLane(
    const PointVector & leftLane, const PointVector & centerLane,
    const PointVector & rightLane) const;

  /**
   * @brief Transform the provided point vector from camera coordinates to car coordinates
   * @param lane vector with points that will be transformed
   * @return Reference to the lane parameter
   */
  PointVector & transformToCarCoordinates(PointVector & lane) const;

private:
  // config of this node insatnce
  TrajectoryConfig mConfig;
  // optional kalman filter used for trajectory smoothing
  LaneKalmanFilter * mKalmanFilter = nullptr;
  // ImageSaver instance used to save debug images
  psaf_shared::ImageSaver * const mImageSaver = nullptr;

#ifdef BUILD_UNIT_TEST
  // friend declaration used for unit test access to private members
  friend class TrajectoryUnitTest_TestStateUpdate_Test;
  friend class TrajectoryUnitTest_TestUnused_Test;
  friend class TrajectoryUnitTest_TestProcessLaneMarkingsEdgeCases_Test;
#endif
};

#endif  // PSAF_TRAJECTORY__TRAJECTORY_NODE_HPP_
