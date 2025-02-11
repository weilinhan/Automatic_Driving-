/**
 * @file trajectory_node.cpp
 * @brief The implementation of the trajectory node
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_trajectory/trajectory_node.hpp"
#include <string>
#include <sstream>
#include <chrono>
#include <filesystem>

#include "psaf_shared/image_saver.hpp"
#include "psaf_shared/image_utils.hpp"

TrajectoryConfig TrajectoryConfig::fromParameters(rclcpp::Node & node)
{
  TrajectoryConfig c;

  c.updateFrequency =
    std::fabs(node.declare_parameter<double>("node_update_frequency", c.updateFrequency));
  c.debugOutput = node.declare_parameter<bool>("debug_output", c.debugOutput);
  c.ignoreState = node.declare_parameter<bool>("ignore_state", c.ignoreState);
  c.showImages = node.declare_parameter<bool>("show_images", c.showImages);
  c.saveImages = node.declare_parameter<bool>("save_images", c.saveImages);
  c.leftLaneOffset = node.declare_parameter<double>("left_lane_offset", c.leftLaneOffset);
  c.centerLaneOffset = node.declare_parameter<double>("center_lane_offset", c.centerLaneOffset);
  c.rightLaneOffset = node.declare_parameter<double>("right_lane_offset", c.rightLaneOffset);
  c.minPointCount = std::abs(node.declare_parameter<int>("min_lane_point_count", c.minPointCount));
  c.minVecLength = std::fabs(node.declare_parameter<double>("min_vec_length", c.minVecLength));
  c.enableKalman = node.declare_parameter<bool>("enable_kalman", c.enableKalman);
  c.kProcessNoiseCoeff = node.declare_parameter<double>(
    "process_noise_coefficient", c.kProcessNoiseCoeff);
  c.kMaxErrorPointDistance = node.declare_parameter<double>(
    "max_error_point_distance", c.kMaxErrorPointDistance);

  // internal kalman filter only allows float values, so we have to narrow here!
  c.kMeasureNoiseCov[0] = node.declare_parameter<double>(
    "measurement_noise_coefficient_x", c.kMeasureNoiseCov[0]);
  c.kMeasureNoiseCov[1] = node.declare_parameter<double>(
    "measurement_noise_coefficient_y", c.kMeasureNoiseCov[1]);
  c.kStateStart[0] = node.declare_parameter<double>("state_start_xp", c.kStateStart[0]);
  c.kStateStart[1] = node.declare_parameter<double>("state_start_yp", c.kStateStart[1]);
  c.kStateStart[2] = node.declare_parameter<double>("state_start_xpp", c.kStateStart[2]);
  c.kStateStart[3] = node.declare_parameter<double>("state_start_ypp", c.kStateStart[3]);

  c.rotMatData[0] = node.declare_parameter<double>("rotation_11", c.rotMatData[0]);
  c.rotMatData[1] = node.declare_parameter<double>("rotation_12", c.rotMatData[1]);
  c.rotMatData[2] = node.declare_parameter<double>("rotation_21", c.rotMatData[2]);
  c.rotMatData[3] = node.declare_parameter<double>("rotation_22", c.rotMatData[3]);
  c.transMatData[0] = node.declare_parameter<double>("translation_1", c.transMatData[0]);
  c.transMatData[1] = node.declare_parameter<double>("translation_2", c.transMatData[1]);
  c.rotMat = cv::Mat(2, 2, CV_64F, c.rotMatData);
  c.transMat = cv::Mat(2, 1, CV_64F, c.transMatData);

  return c;
}

std::ostream & operator<<(std::ostream & os, const TrajectoryConfig & cfg)
{
  os << "==== Configuration ====\n";
  os << "- updateFrequency: " << cfg.updateFrequency << "\n";
  os << "- debugOutput: " << std::boolalpha << cfg.debugOutput << "\n";
  os << "- minPointCount: " << cfg.minPointCount << "\n";
  os << "- centerLaneOffset: " << cfg.centerLaneOffset << "\n";
  os << "- leftLaneOffset: " << cfg.leftLaneOffset << "\n";
  os << "- rightLaneOffset: " << cfg.rightLaneOffset << "\n";
  os << "- transMat: " << cfg.transMat << "\n";
  os << "- rotMat: " << cfg.rotMat << "\n";
  os << "- minVecLength: " << cfg.minVecLength << "\n";
  os << "- showImages: " << std::boolalpha << cfg.showImages << "\n";
  os << "- saveImages: " << std::boolalpha << cfg.saveImages << "\n";
  os << "- ignoreState: " << std::boolalpha << cfg.ignoreState << "\n";
  os << "- enableKalman: " << std::boolalpha << cfg.enableKalman << "\n";
  if (cfg.enableKalman) {
    os << "- kProcessNoiseCoeff: " << cfg.kProcessNoiseCoeff << "\n";
    os << "- kMaxErrorPointDistance: " << cfg.kMaxErrorPointDistance << "\n";
    os << "- kMeasureNoiseCov: [" << cfg.kMeasureNoiseCov[0] << ", " << cfg.kMeasureNoiseCov[1] <<
      "]\n";
    os << "- kStateStart: [" << cfg.kStateStart[0] << ", " << cfg.kStateStart[1] << ", " <<
      cfg.kStateStart[2] << ", " << cfg.kStateStart[3] << "]\n";
  }
  os << "==== End of Cfg =======";

  return os;
}

TrajectoryNode::TrajectoryNode(psaf_shared::ImageSaver * imageSaver)
:  // initalize used interface
  TrajectoryInterface(
    TRAJECTORY_NODE_NAME,
    OBJECT_TOPIC,
    STATE_TOPIC,
    LANE_MARKINGS_TOPIC,
    TRAJECTORY_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})), mImageSaver(imageSaver)
{
}

TrajectoryNode::~TrajectoryNode()
{
  if (mKalmanFilter != nullptr) {
    delete mKalmanFilter;
    mKalmanFilter = nullptr;
  }
}

void TrajectoryNode::applyConfig(const TrajectoryConfig & cfg)
{
  mConfig = cfg;

  if (mConfig.enableKalman) {
    // delete old filter first
    if (mKalmanFilter != nullptr) {
      delete mKalmanFilter;
      mKalmanFilter = nullptr;
    }

    // construct new filter
    mKalmanFilter = new LaneKalmanFilter(
      mConfig.kProcessNoiseCoeff, mConfig.kMaxErrorPointDistance,
      mConfig.kMeasureNoiseCov, mConfig.kStateStart);
  }

  // recalculate member mats
  mConfig.rotMat = cv::Mat(2, 2, CV_64F, mConfig.rotMatData);
  mConfig.transMat = cv::Mat(2, 1, CV_64F, mConfig.transMatData);

  if (mConfig.debugOutput) {
    std::cout << mConfig << std::endl;
  }
}

TrajectoryConfig TrajectoryNode::getConfig() const
{
  return mConfig;
}

void TrajectoryNode::update()
{
  // callback, called in update_frequency intervals
}

void TrajectoryNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  // update local copy of state machine state
  // see state_definitions.hpp for values
  this->state = state->data;
}

void TrajectoryNode::processObstacle(libpsaf_msgs::msg::Obstacle::SharedPtr p)
{
  // we dont implement obstacle detection
  (void)p;
}

void TrajectoryNode::processLaneMarkings(libpsaf_msgs::msg::LaneMarkings::SharedPtr p)
{
  // check if we are in driving state, otherwise we dont need to process markings
  // if we ignore state skip this check
  if (!mConfig.ignoreState) {
    if (state != 10) {
      if (mConfig.debugOutput) {
        std::cout << "Got lane markings, but not in STATE_DRIVING, skipping...\n";
      }
      return;
    }
  }

  // copy all lane markings and get required params
  PointVector centerLane = transformToCarCoordinates(p.get()->center_lane);
  PointVector leftLane = transformToCarCoordinates(p.get()->left_lane);
  PointVector rightLane = transformToCarCoordinates(p.get()->right_lane);

  // check that atleast on line contains enough points
  if (centerLane.size() < mConfig.minPointCount &&
    leftLane.size() < mConfig.minPointCount &&
    rightLane.size() < mConfig.minPointCount)
  {
    if (mConfig.debugOutput) {
      std::cout <<
        "Failed to calculate trajectory: all LaneMarkings contain less than minPointCount("
                << mConfig.minPointCount << ") points!\n";
    }
    return;
  }

  LaneType choosenLaneType = chooseReferenceLane(leftLane, centerLane, rightLane);
  PointVector referenceLane;
  switch (choosenLaneType) {
    case LaneType::LEFT_LANE: referenceLane = leftLane; break;
    case LaneType::CENTER_LANE: referenceLane = centerLane;break;
    case LaneType::RIGHT_LANE: referenceLane = rightLane;break;
  }

  // use kalman filter to smooth lane if enabled
  if (mConfig.enableKalman && mKalmanFilter != nullptr) {
    mKalmanFilter->smoothLane(referenceLane);
  }

  // calculate our trajectory
  PointVector traj = offsetLaneToTrajectory(referenceLane, choosenLaneType);

  if (traj.size() == 0) {
    // dont publish empty trajectory, error instead
    std::cout << "Failed to calculate trajectory: trajectory.size = 0" << std::endl;
    return;
  }

  if (mConfig.debugOutput) {
    std::string lane;
    switch (choosenLaneType) {
      case LaneType::CENTER_LANE:
        lane = "center";
        break;
      case LaneType::RIGHT_LANE:
        lane = "right";
        break;
      case LaneType::LEFT_LANE:
        lane = "left";
        break;
    }
    // print trajectory
    std::cout << "Calculated trajectory with " << traj.size() << " points from " << lane <<
      " lane: ";
    std::cout << "trajectory from (" << traj.at(0).x << ", " << traj.at(0).y << ") to (" <<
      traj.back().x << ", " << traj.back().y << ")\n";
  }

  publishTrajectory(traj);

  if (mConfig.showImages || mConfig.saveImages) {
    // create output image to our specs
    auto image = psaf_shared::plot_points(
      {traj, leftLane, centerLane, rightLane},
      {psaf_shared::colors::red,
        psaf_shared::colors::white,
        psaf_shared::colors::green,
        psaf_shared::colors::white},
      2.5, 0.5, true, cv::ROTATE_90_COUNTERCLOCKWISE, cv::Size(480, 640));

    if (mConfig.showImages) {
      psaf_shared::wrapped_imshow("Trajectory", image);
    }

    if (mConfig.saveImages && mImageSaver != nullptr) {
      mImageSaver->save(image, "trajectory");
    }
  }
}

TrajectoryNode::PointVector TrajectoryNode::offsetLaneToTrajectory(
  const PointVector & lane, const LaneType laneType) const
{
  PointVector traj;
  if (!lane.empty()) {
    for (size_t i = 0; i < lane.size() - 1; ++i) {
      auto & current = lane.at(i);
      auto & next = lane.at(i + 1);

      double dx = next.x - current.x;
      double dy = next.y - current.y;

      double length = std::sqrt(dx * dx + dy * dy);  // Länge des Verbindungsvektors

      // check length is sensible
      if (length < mConfig.minVecLength) {
        std::cout << "Calculated vector length is to small (" << length << ") skipping point!\n";
        continue;
      }

      double orthoX = -dy / length;  // Normalisierung und Orthogonalität
      double orthoY = dx / length;

      double scale;
      switch (laneType) {
        case LaneType::CENTER_LANE:
          scale = mConfig.centerLaneOffset;
          break;
        case LaneType::RIGHT_LANE:
          scale = mConfig.rightLaneOffset;
          break;
        case LaneType::LEFT_LANE:
          scale = mConfig.leftLaneOffset;
          break;
        default:
          // should never happen, so we throw error
          throw std::runtime_error("Invalid Lane Type passed to offsetLaneToTrajectory");
      }

      orthoX *= scale;
      orthoY *= scale;

      geometry_msgs::msg::Point newPoint;
      newPoint.x = current.x + dx / 2 + orthoX;
      newPoint.y = current.y + dy / 2 + orthoY;
      newPoint.z = 0;  // heigth is always 0
      traj.push_back(newPoint);
    }
  }

  return traj;
}

TrajectoryNode::PointVector & TrajectoryNode::transformToCarCoordinates(
  PointVector & lane) const
{
  for (auto & point : lane) {
    cv::Mat pointMat(2, 1, CV_64F);
    pointMat.at<double>(0, 0) = point.x;
    pointMat.at<double>(1, 0) = point.y;

    pointMat = mConfig.rotMat * pointMat + mConfig.transMat;

    point.x = pointMat.at<double>(0, 0);
    point.y = pointMat.at<double>(1, 0);
    point.z = 0;
  }

  return lane;
}

TrajectoryNode::LaneType TrajectoryNode::chooseReferenceLane(
  const PointVector & leftLane,
  const PointVector & centerLane,
  const PointVector & rightLane) const
{
  // store lane sizes with their lane types as tuples
  std::tuple<size_t, LaneType> left(leftLane.size(), LaneType::LEFT_LANE);
  std::tuple<size_t, LaneType> right(rightLane.size(), LaneType::RIGHT_LANE);
  std::tuple<size_t, LaneType> center(centerLane.size(), LaneType::CENTER_LANE);
  std::vector<std::tuple<size_t, LaneType>> sizes = {right, center, left};

  // use stable sort so relativ order of equal sizes stays the same
  std::stable_sort(
    sizes.begin(), sizes.end(), [](const std::tuple<size_t, LaneType> & t1,
    const std::tuple<size_t, LaneType> & t2) -> bool {
      // compare tuples by their size
      return std::get<0>(t1) > std::get<0>(t2);
    });

  return std::get<1>(sizes.front());
}
