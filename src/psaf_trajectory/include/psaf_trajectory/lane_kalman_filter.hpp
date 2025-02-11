/**
 * @file lane_kalman_filter.hpp
 * @brief Definition of the KalmanFilter class
 * @author PSAF
 * @date 2024-01-29
 */
#ifndef PSAF_TRAJECTORY__LANEKALMANFILTER_HPP_
#define PSAF_TRAJECTORY__LANEKALMANFILTER_HPP_

#include <vector>

#include "opencv4/opencv2/opencv.hpp"
#include "geometry_msgs/msg/point.hpp"

/**
 * @class LaneKalmanFilter
 * @brief Encapsulates an cv::KalmanFilter and provides a function to smooth lane markings
 */
class LaneKalmanFilter
{
public:
/**
 * @brief Construct a new LaneKalmanFilter object
 * @param processNoise TODO(sven.kalmbach): ask @sven.vollstädt about kf params
 * @param maxErrorPointDistance max distance of a measurement from to expected point for it to be used to update kf
 * @param measureNoiseCov TODO(sven.kalmbach): ask @sven.vollstädt about kf params
 * @param preState TODO(sven.kalmbach): ask @sven.vollstädt about kf params
 */
  LaneKalmanFilter(
    const float processNoise, const float maxErrorPointDistance,
    const float (& measureNoiseCov)[2], const float (& preState)[4]);

  /**
   * @brief Smooth a series point vector using a kalman filter
   * @details Kalman filter is reset each time this function is called
   * @param lane The lane which will be smoothed by the filter, should not be empty.
   * @return false if there were outliers or the lane is empty, else true
   */
  bool smoothLane(std::vector<geometry_msgs::msg::Point> & lane);

  // deleted copy constructor
  LaneKalmanFilter(const LaneKalmanFilter &) = delete;
  // delted assign operator
  LaneKalmanFilter & operator=(LaneKalmanFilter const &) = delete;

protected:
  /**
   * @brief Reset the kalman filter to a initial state
   * @param point point used to set the initial filter state
   */
  void resetFilter(const geometry_msgs::msg::Point & point);

  /**
   * @brief Process a single point/measurement with the kf
   * @param point input point which will be corrected (even if its an outlier)
   * @return false if the point is an outlier else true
   */
  bool filterPoint(geometry_msgs::msg::Point & point);

private:
  // internal opencv kalman filter implementation
  cv::KalmanFilter mKf;

  // max distance of a measurement from to expected point for it to be used to update kf
  const float mMaxErrorPointDistance;

  // parameters for kalman filter
  const float mProcessNoise;
  const float mPreState[6] = {0};
  const float mMeasureNoiseCov[4] = {0};
};

#endif  // PSAF_TRAJECTORY__LANEKALMANFILTER_HPP_
