/**
 * @file startbox_node.hpp
 * @brief the definition of the startbox node
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STARTBOX__STARTBOX_NODE_HPP_
#define PSAF_STARTBOX__STARTBOX_NODE_HPP_

#include <zbar.h>
#include <string>
#include <vector>

#include "opencv4/opencv2/opencv.hpp"
#include "libpsaf/interface/start_box_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"

/**
 * @brief Struct class used to encapsulate all configs for a StartBoxConfig
 */
struct StartBoxConfig
{
  // update frequency of this node
  double updateFrequency = 60.0;
  // whether to ignore qr code on the startbox
  bool ignoreQr = false;
  // whether to ignore the distance sensor values
  bool ignoreDistance = false;
  // distance in meters at which the start box is considered to be open
  double triggerDistance = 0.3;

  /**
   * @brief Load a StartBoxConfig from parameters using the provided node
   * @param node the node to declare the parameters on
   * @return StartBoxConfig the created instance
   */
  static StartBoxConfig fromParameters(rclcpp::Node & node);
};

/**
 * @class StartBoxNode
 * @implements StartBoxInterface
 * @brief Detects the opening of the startbox
 * @details This class is the main class of the startbox node. It is responsible for the
 * detection of the opening of the startbox. This can be done by detecting the QR-code, the STOP-sign
 * or the opening of the gate itself.
 */
class StartBoxNode : public libpsaf::StartBoxInterface
{
public:
  StartBoxNode();
  // The content of the last read QR-code
  std::string last_read_qr_{"INIT"};
  // At least one QR-code has been read
  bool detected_at_least_once_{false};
  // The door is presumed to be open
  bool is_open_{false};
  // The current state of the state machine
  int current_state_{0};
  // The last read sensor value by the US sensor
  double last_received_distance_{0.0};
  // Counter for US ranges greater than 0.3m
  int us_msg_counter_{0};
  // The amount of images where no QR-code was detected
  int no_qr_msg_counter_{0};

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

  /**
 * @brief Apply the StartBoxConfig to this node
 * @param cfg the StartBoxConfig to apply to this node
 */
  void applyConfig(const StartBoxConfig & cfg);

protected:
  /**
   * @brief Callback Method for the state
   * @param[in] state the current state of the State Machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state);

  /**
   * The callback method for the image
   * @param[in] img the image
   * @param[in] sensor the position of the image topic in the topic vector
   */
  void processImage(cv::Mat & img, int sensor) final;

  /**
   * @brief Callback Method for the Ultrasonic Sensors
   * @param[in] p the measured distance in cm
   * @param[in] sensor the position of the sensor in the topic vector
   */
  void updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor);

  /**
   * @brief Search for a QR-code in the image and read it
   * @param[in] img the input image
   */
  void readQR(cv::Mat & img);

private:
  // config of this node instance
  StartBoxConfig mConfig;
};

#endif  // PSAF_STARTBOX__STARTBOX_NODE_HPP_
