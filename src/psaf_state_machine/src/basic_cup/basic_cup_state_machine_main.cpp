/**
 * @file basic_cup_state_machine_main.cpp
 * @brief the main method for the startbox. This function gets called by the launch file
 * @author PSAF
 * @date 2022-06-01
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_state_machine/basic_cup/basic_cup_state_machine_node.hpp"

/**
 * @brief Advanced State Machine for the masters carolo cup
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<BasicCupStateMachineNode> node = std::make_shared<BasicCupStateMachineNode>();
  rclcpp::WallRate rate(node->declare_parameter<double>("update_frequency"));
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
}
