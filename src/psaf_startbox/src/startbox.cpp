/**
 * @file startbox.cpp
 * @brief the main method for the startbox. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_startbox/startbox_node.hpp"

/**
* Main: Start the Startbox Node
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create this node
  std::shared_ptr<StartBoxNode> node = std::make_shared<StartBoxNode>();

  // load this nodes config from .yaml and apply it
  const auto cfg = StartBoxConfig::fromParameters(*node);
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
