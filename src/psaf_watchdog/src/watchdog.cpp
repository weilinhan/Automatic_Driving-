/**
 * @file watchdog.cpp
 * @brief the main method for the watchdog node. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_watchdog/watchdog_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create node with default config
  std::shared_ptr<WatchdogNode> node = std::make_shared<WatchdogNode>();
  // load config from node params and apply it
  const auto config = WatchdogConfig::fromParameters(*node);
  node->applyConfig(config);

  // load rate from config and start spinning node
  rclcpp::WallRate rate(config.updateFrequency);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
