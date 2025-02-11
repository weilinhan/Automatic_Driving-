#include "psaf_trajectory/lane_kalman_filter.hpp"

LaneKalmanFilter::LaneKalmanFilter(
  const float processNoise,
  const float maxErrorPointDistance,
  const float (& measureNoiseCov)[2],
  const float (& preState)[4]
)
: mMaxErrorPointDistance(maxErrorPointDistance),
  mProcessNoise(processNoise),
  mPreState{0, 0, preState[0], preState[1], preState[2], preState[3]},
  mMeasureNoiseCov{measureNoiseCov[0], 0, 0, measureNoiseCov[1]}
{
}

bool LaneKalmanFilter::smoothLane(std::vector<geometry_msgs::msg::Point> & lane)
{
  // error flags
  bool hasOutliers = false;
  bool isEmpty = false;

  // only process non-empty lnaes
  if (!lane.empty()) {
    // reset kalman for each lane, since lanes should not influence each other
    resetFilter(lane.front());

    // filter each point with calman filter
    for (auto & point : lane) {
      // if we get a true retval point is out of range
      if (filterPoint(point)) {
        hasOutliers = true;
      }
    }
  } else {
    isEmpty = true;
  }

  return hasOutliers || isEmpty;
}

void LaneKalmanFilter::resetFilter(const geometry_msgs::msg::Point & point)
{
  // get a fresh filter object
  mKf = cv::KalmanFilter(6, 2);

  mKf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
    1, 0, 1, 0, 0, 0,
    0, 1, 0, 1, 0, 0,
    0, 0, 1, 0, 1, 0,
    0, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1);
  mKf.processNoiseCov = (cv::Mat_<float>(6, 6) <<
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 0.2, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1) * mProcessNoise;

  mKf.measurementMatrix = cv::Mat_<float>::eye(2, 6);
  mKf.errorCovPre = cv::Mat_<float>::eye(6, 6);

  // use << initialization since data is const
  mKf.measurementNoiseCov = (cv::Mat_<float>(2, 2)
    << mMeasureNoiseCov[0], mMeasureNoiseCov[1], mMeasureNoiseCov[2], mMeasureNoiseCov[3]);
  // initial state based on first point
  mKf.statePre = (cv::Mat_<float>(6, 1)
    << point.x, point.y, mPreState[2], mPreState[3], mPreState[4], mPreState[5]);
  // we need to correct kalman once in order to get correct output
  mKf.correct((cv::Mat_<float>(2, 1) << point.x, point.y));
}

bool LaneKalmanFilter::filterPoint(geometry_msgs::msg::Point & point)
{
  bool outlier = false;
  // predict the next point
  cv::Mat prediction = mKf.predict();

  // calculate distance between point and prediction
  float distance =
    std::sqrt(
    std::pow(point.x - prediction.at<float>(0), 2)
    +
    std::pow(point.y - prediction.at<float>(1), 2)
    );

  // update kalman, if measurement is withing range of prediction
  outlier = distance >= mMaxErrorPointDistance;

  if (!outlier) {
    cv::Mat pointMat = (cv::Mat_<float>(2, 1) << point.x, point.y);
    mKf.correct(pointMat);
  }

  // extract filtered point to update
  point.x = mKf.statePost.at<float>(0);
  point.y = mKf.statePost.at<float>(1);

  return outlier;
}
