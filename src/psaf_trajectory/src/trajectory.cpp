/**
 * @file trajectory.cpp
 * @brief the main method for the trajectory node. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_trajectory/trajectory_node.hpp"
#include "psaf_configuration/configuration.hpp"
#include "psaf_shared/image_saver.hpp"

/**
* Main: Start the Startbox Node
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create this nodes imagesaver
  psaf_shared::ImageSaver imageSaver(TRAJECTORY_NODE_NAME);
  // create this node
  std::shared_ptr<TrajectoryNode> node = std::make_shared<TrajectoryNode>(&imageSaver);

  // load this nodes config from .yaml and apply it
  const auto cfg = TrajectoryConfig::fromParameters(*node);
  node->applyConfig(cfg);

  // create the rate timer for this node from cfg frequency
  rclcpp::WallRate rate(cfg.updateFrequency);

  // spin the node
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
