/**
 * @file sign_detection.cpp
 * @brief the main method for the sign detection. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_sign_detection/sign_detection_node.hpp"
#include "psaf_configuration/configuration.hpp"
#include "psaf_shared/image_saver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // create imagesaver instance for node
  psaf_shared::ImageSaver imageSaver(SIGN_DETECTION_NODE_NAME);

  // create node pointer
  std::shared_ptr<SignDetectionNode> node = std::make_shared<SignDetectionNode>(&imageSaver);

  // load node config from parameters
  const auto cfg = SignDetectionConfig::fromParameters(*node);

  // apply config to node and wallrate
  node->applyConfig(cfg);
  rclcpp::WallRate rate(cfg.updateFrequency);

  // spin the node
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
