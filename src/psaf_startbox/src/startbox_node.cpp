/**
 * @file startbox_node.cpp
 * @brief the implementation of the startbox node
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_startbox/startbox_node.hpp"
#include <vector>
#include <string>
#include <iostream>

StartBoxConfig StartBoxConfig::fromParameters(rclcpp::Node & node)
{
  StartBoxConfig c;

  c.updateFrequency =
    std::fabs(node.declare_parameter<double>("update_frequency", c.updateFrequency));
  c.ignoreDistance = node.declare_parameter<bool>("ignore_distance", c.ignoreDistance);
  c.ignoreQr = node.declare_parameter<bool>("ignore_qr", c.ignoreQr);
  c.triggerDistance = std::fabs(
    node.declare_parameter<double>("trigger_distance", c.triggerDistance));

  return c;
}

StartBoxNode::StartBoxNode()
: StartBoxInterface(
    STARTBOX_NODE_NAME,
    NBR_OF_CAMS,
    CAM_TOPICS,
    NBR_OF_US_SENSORS,
    US_TOPICS,
    STATE_TOPIC,
    STATUS_INFO_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
)
{
}

void StartBoxNode::applyConfig(const StartBoxConfig & cfg)
{
  mConfig = cfg;
  if (mConfig.ignoreDistance && mConfig.ignoreQr) {
    throw std::runtime_error(
            "Ignoring QR and Distance is not allowed, please change config.yaml.");
  }
}

void StartBoxNode::processImage(cv::Mat & img, int sensor)
{
  // 0 means RGB image
  if (sensor == 0 && !mConfig.ignoreQr) {
    // Check if the image is not empty
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Image is empty");
      return;
    }
    // Check if the image is in the correct format 640 x 480
    // Otherwise resize
    if (img.cols != 640 || img.rows != 480) {
      cv::resize(img, img, cv::Size(640, 480));
    }

    // Convert to Gray
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    // Detect the QR code
    readQR(img);
  }
}


void StartBoxNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  current_state_ = state->data;
}

void StartBoxNode::update()
{
  if (is_open_) {
    libpsaf_msgs::msg::StatusInfo msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "";
    msg.type = libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN;
    publishStatus(msg);
  }

  if (current_state_ != 0) {
    // shutdown the node since it is not needed anymore
    rclcpp::shutdown();
  }
}

void StartBoxNode::updateSensorValue(sensor_msgs::msg::Range::SharedPtr p, int sensor)
{
  // Only consider sensor 0, since this is the front US sensor
  if (sensor == 0 && !mConfig.ignoreDistance) {
    // Ignore 0.0 values. Those are just noise
    if (p->range > 0.0) {
      last_received_distance_ = p->range;
      // Assume more than mTriggerDistance cm is door open
      if (p->range >= mConfig.triggerDistance) {
        us_msg_counter_++;
      } else {
        us_msg_counter_ = 0;
      }
    }
    // If 10 consecutive measurements are over mTriggerDistance cm, then the door is open
    if (us_msg_counter_ > 10) {
      is_open_ = true;
    }
  }
}

void StartBoxNode::readQR(cv::Mat & image)
{
  zbar::Image image_to_zbar(image.cols, image.rows, "Y800", image.data, image.cols * image.rows);
  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  int n = scanner.scan(image_to_zbar);
  if (n > 0) {
    for (zbar::Image::SymbolIterator symbol = image_to_zbar.symbol_begin();
      symbol != image_to_zbar.symbol_end(); ++symbol)
    {
      if (symbol->get_type() == zbar::ZBAR_QRCODE) {
        last_read_qr_ = symbol->get_data();

        if (last_read_qr_ == "STOP") {
          detected_at_least_once_ = true;
          no_qr_msg_counter_ = 0;
        }
        break;
      }
    }
  } else {
    last_read_qr_ = "";
    no_qr_msg_counter_++;
  }

  if (no_qr_msg_counter_ > 10) {
    is_open_ = true;
  }
}
