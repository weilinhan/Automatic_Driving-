/**
 * @file test_util.hpp
 * @brief Header file containing useful functions for testing.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef TEST_UTIL_HPP_
#define TEST_UTIL_HPP_

#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/filesystem_helper.hpp"

#include "psaf_state_machine/basic_cup/basic_cup_state_machine_node.hpp"

/**
 * @class PubSubDummy
 * Dummy Publisher Subscriber Class to Test the Response of a node to a certain input
 * @tparam MessageP the type of the published message
 * @tparam MessageS the type of the subscribed message
 */
template<typename MessageP, typename MessageS>
class PubSubDummy : public rclcpp::Node
{
public:
  PubSubDummy(
    std::string pub_topic_name,
    std::string sub_topic_name,
    std::vector<MessageP> & data,
    std::shared_ptr<BasicCupStateMachineNode> node,
    std::chrono::milliseconds rate = std::chrono::milliseconds(300))
  : Node("DummyPubSub"), pub_topic_(pub_topic_name), sub_topic_(sub_topic_name),
    data_(data), node(node), rate_(rate), finished_(false), can_publish(true), index(0) {}

  /**
  * @brief Function to start the publisher and subscriber threads
  * @details In this method all of the specified messages will be send. This happens sequentially.
  * A new message is only send if the answer to the previous one was received.
  */
  void run()
  {
    index = 0;
    rclcpp::QoS qos(rclcpp::QoS(rclcpp::KeepLast{10}));
    publisher_ = this->create_publisher<MessageP>(pub_topic_, qos);

    subscriber_ = this->create_subscription<MessageS>(
      sub_topic_, qos,
      [this](std::shared_ptr<MessageS> msg) {
        results_.push_back(*msg);
        can_publish = true;
        if (results_.size() == data_.size()) {
          finished_ = true;
        }
      });

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(40), [this]() {
        if (can_publish) {
          publisher_->publish(data_.at(index));
          index++;
          can_publish = false;
        }
        rclcpp::spin_some(node);
      });
  }

  /**
   * @brief Method to determine the end of a run
   * @return true if as many answers as messages were received
   */
  bool hasFinished()
  {
    return finished_;
  }

  /**
   * Returns the collected Messages during the run
   * @return the collected Messages
   */
  std::vector<MessageS> & getResult()
  {
    return results_;
  }

private:
  /** The subscriber */
  typename rclcpp::Subscription<MessageS>::SharedPtr subscriber_;

  /** The Publisher */
  typename rclcpp::Publisher<MessageP>::SharedPtr publisher_;

  /** Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /** The topic names */
  std::string pub_topic_;
  std::string sub_topic_;

  /** Input data */
  std::vector<MessageP> data_;

  std::shared_ptr<BasicCupStateMachineNode> node;

  /** Output data */
  std::vector<MessageS> results_;

  /** Timer speed */
  std::chrono::milliseconds rate_;
  /** Helper variables**/
  bool finished_;
  bool can_publish;
  int index;
};


#endif  // TEST_UTIL_HPP_
