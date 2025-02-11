/**
 * @file lane_detection.cpp
 * @brief the main method for the lane_detection. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_lane_detection/lane_detection_node.hpp"
#include "psaf_configuration/configuration.hpp"
#include "psaf_shared/image_saver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create this nodes imagesaver
  psaf_shared::ImageSaver imageSaver(LANE_DETECTION_NODE_NAME);
  // create this node
  std::shared_ptr<LaneDetectionNode> node = std::make_shared<LaneDetectionNode>(&imageSaver);
  // load this nodes config from .yaml and apply it
  const auto cfg = LaneDetectionConfig::fromParameters(*node);
  node->applyConfig(cfg);

  // create the rate timer for this node from cfg frequency
  rclcpp::WallRate rate(cfg.updateFrequency);

  // spin the node
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }
}
