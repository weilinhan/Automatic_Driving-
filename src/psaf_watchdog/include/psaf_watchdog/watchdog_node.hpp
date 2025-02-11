/**
 * @file watchdog_node.hpp
 * @brief The definition of the watchdog node
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_WATCHDOG__WATCHDOG_NODE_HPP_
#define PSAF_WATCHDOG__WATCHDOG_NODE_HPP_

#include <string>
#include <limits>
#include "libpsaf/interface/watchdog_interface.hpp"
#include "psaf_configuration/configuration.hpp"

/**
 * @brief Struct class used to encapsulate all configs for a WatchdogNode
 */
struct WatchdogConfig
{
  // this nodes update frequency
  double updateFrequency = 30.0;
  // distance to stop at in meters
  double stopDistance = 0.2;
  // max valid sensor value
  double sensorMaxThreshold = 0.9;
  // min valid sensor value
  double sensorMinThreshold = 0.02;
  // wheter to provide debug ouput or not
  bool debugOutput = false;

  /**
   * @brief Load a WatchDogConfig from parameters using the provided node
   * @param node used to load the config
   * @return WatchdogConfig the loaded config
   */
  static WatchdogConfig fromParameters(rclcpp::Node & node);

  /**
   * @brief Output this config to an ostream
   * @param os the ostream to output to
   * @param cfg the config to output to the stream
   * @return std::ostream& reference to the input ostream
   */
  friend std::ostream & operator<<(std::ostream & os, const WatchdogConfig & cfg);
};

/**
 * @class WatchdogNode
 * @implements WatchdogInterface
 * @brief Monitor certain hardware components for faults
 * @details This class is the watchdog node. It is responsible for
 * monitoring various parameters of the system and issue error and warning
 * messages if the system is not behaving as expected.
 * This includes failures of the camera, US sensors, and motors
 * INPUT: camera image, us sensor data, speed and steering data. current state
 * OUTPUT: Error Message
 */
class WatchdogNode : public libpsaf::WatchdogInterface
{
public:
  /**
   * @brief Construct a new Watchdog Node object
   */
  WatchdogNode();

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

  /**
   * @brief Apply the provided WatchdogConfig to this node
   * @param config the config to apply to this node
   */
  void applyConfig(const WatchdogConfig & config);

  /**
   * @brief Get this nodes current WatchdogConfig
   * @return WatchdogConfig the current WatchdogConfig
   */
  WatchdogConfig getConfig() const;

protected:
  /**
   * @brief Callback Method for the camera images
   * @param[in] image the image
   * @param[in] sensor the position of the topic in the topic vector
   */
  void processImage(cv::Mat & image, int sensor) final;

  /**
   * @brief Callback Method for the speed of the car
   * @param[in] speed the speed of the car in cm/s
   */
  void updateSpeed(std_msgs::msg::Int16::SharedPtr speed) override;

  /**
   * The callback Method for the steering angle
   * @param[in] steering the steering angle
   */
  void updateSteering(std_msgs::msg::Int16::SharedPtr steering) override;

  /**
   * @brief Callback Method for the ultrasonic sensors
   * @param[in] p the distance of the sensor in cm
   * @param[in] sensor the position of the topic in the topic vector
   */
  void updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor) override;

  /**
   * Callback Method for the current state
   * @param[in] state the current state of the State Machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state);

private:
  // config used by this node
  WatchdogConfig mConfig;

#ifdef BUILD_UNIT_TEST
  friend class WatchdogUnitTest_TestStateUpdate_Test;
  friend class WatchdogUnitTest_TestSimpleStop_Test;
  friend class WatchdogUnitTest_TestOutOfRange_Test;
#endif
};

#endif  // PSAF_WATCHDOG__WATCHDOG_NODE_HPP_
