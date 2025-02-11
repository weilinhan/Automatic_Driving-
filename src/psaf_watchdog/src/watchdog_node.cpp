/**
 * @file watchdog_node.cpp
 * @brief The implementation of the watchdog node
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_watchdog/watchdog_node.hpp"
#include <string>
#include <vector>

WatchdogConfig WatchdogConfig::fromParameters(rclcpp::Node & node)
{
  WatchdogConfig c;

  c.updateFrequency = std::fabs(
    node.declare_parameter<double>("update_frequency", c.updateFrequency));
  c.stopDistance = std::fabs(node.declare_parameter<double>("stop_distance", c.stopDistance));
  c.sensorMaxThreshold = std::fabs(
    node.declare_parameter<double>("sensor_max_threshold", c.sensorMaxThreshold));
  c.sensorMinThreshold = std::fabs(
    node.declare_parameter<double>("sensor_min_threshold", c.sensorMinThreshold));
  c.debugOutput = node.declare_parameter<bool>("debug_output", c.debugOutput);

  return c;
}

std::ostream & operator<<(std::ostream & os, const WatchdogConfig & cfg)
{
  os << "==== Configuration ====\n";
  os << "- updateFrequency: " << cfg.updateFrequency << "\n";
  os << "- stopDistance: " << cfg.stopDistance << "\n";
  os << "- sensorMaxThreshold: " << cfg.sensorMaxThreshold << "\n";
  os << "- sensorMinThreshold: " << cfg.sensorMinThreshold << "\n";
  os << "- debugOutput: " << std::boolalpha << cfg.debugOutput << "\n";
  os << "==== End of Cfg =======";

  return os;
}

WatchdogNode::WatchdogNode()
: WatchdogInterface(
    WATCHDOG_NODE_NAME,
    NBR_OF_CAMS,
    NBR_OF_US_SENSORS,
    CAM_TOPICS,
    US_TOPICS,
    GET_SPEED_TOPIC,
    GET_STEERING_TOPIC,
    STATE_TOPIC,
    WATCHDOG_ERROR_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
)
{
}

void WatchdogNode::applyConfig(const WatchdogConfig & config)
{
  mConfig = config;

  // print cfg if required
  if (mConfig.debugOutput) {
    std::cout << mConfig << std::endl;
  }
}

WatchdogConfig WatchdogNode::getConfig() const
{
  return mConfig;
}

void WatchdogNode::processImage(cv::Mat & image, int sensor)
{
  (void) image;
  (void) sensor;
}

void WatchdogNode::updateSpeed(std_msgs::msg::Int16::SharedPtr speed)
{
  (void) speed;
}

void WatchdogNode::updateSteering(std_msgs::msg::Int16::SharedPtr steering)
{
  (void) steering;
}

void WatchdogNode::updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor)
{
  auto msg = p.get();

  // we only consider front sensor
  if (sensor != 0) {
    return;
  }

  // set min and max range from parameter if msg doesnt contain on
  if (msg->min_range == 0) {
    msg->min_range = mConfig.sensorMinThreshold;
  }

  if (msg->max_range == 0) {
    msg->max_range = mConfig.sensorMaxThreshold;
  }

  if (mConfig.debugOutput) {
    std::cout << "Got new sensor message:"
              << " range=" << msg->range << ", min_range=" << msg->min_range << ", max_range=" <<
      msg->max_range << std::endl;
  }

  double sensorDistance = msg->range;

  // dont process invalid values
  if (sensorDistance < msg->min_range || sensorDistance > msg->max_range) {
    if (mConfig.debugOutput) {
      std::cout << "Didn't consider range as it was outside of min_range/max_range!" << std::endl;
    }
    return;
  }

  // check if distance is to close
  if (sensorDistance < mConfig.stopDistance) {
    // only publish error in driving state
    if (state == 10) {
      publishErrorMessage(2, "Watchdog: Obstacle detected!");

      std::cout << "sensorDistance=" << sensorDistance << " < mStopDistance=" <<
        mConfig.stopDistance <<
        ", publishing error!" << std::endl;
    }
  }
}

void WatchdogNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  this->state = state->data;
  if (mConfig.debugOutput) {
    std::cout << "Got new state: " << state << std::endl;
  }
}

void WatchdogNode::update()
{
  // nothing to do, distance check is handled in callback
}
