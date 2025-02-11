/**
 * @file integration_test.hpp
 * @brief the integration test for the basic cup state machine.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef STATE_MACHINE__INTEGRATION_TEST_HPP_
#define STATE_MACHINE__INTEGRATION_TEST_HPP_

#include <vector>
#include <memory>

#include "psaf_state_machine/basic_cup/basic_cup_state_machine_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include "std_msgs/msg/int64.hpp"

#define STRESS_TEST_COUNTER 10

/**
 * Generate a random sequence of transitions.
 * @param[in] node the node under test
 * @param[in] n the number of transitions to generate
 * @param[in] expected the expected sequence of states
 * @param[in,out] result the actual sequence of states
 * @param[in,out] transition_sequence the sequence of transitions
 */
void generate_random_test_case(
  std::shared_ptr<BasicCupStateMachineNode> node,
  int n, std::vector<int> & expected,
  std::vector<std_msgs::msg::Int64> & result,
  std::vector<int> & transition_sequence);


/**
 * Simulates a button press to set the discipline
 * @param[in] node the state machine, necessary to spin the node
 * @param[in] button the value of the button, should be [0,1]
 */
void publish_button_once(std::shared_ptr<BasicCupStateMachineNode> node, int button);

/**
 * Publish an error message with the given type
 * @param[in] node the state machine, necessary for spinning
 * @param[in] error the error type as declared in libpsaf
 */
void publish_error_once(std::shared_ptr<BasicCupStateMachineNode> node, int error);

/**
 * Simulates a drive
 * @param[in] node the StateMachine Node. Necessary to spin
 * @param[in] data the data to be published
 * @return the result = received messages from the StateMachine
 */
std::vector<std_msgs::msg::Int64> manage_run(
  std::shared_ptr<BasicCupStateMachineNode> node,
  std::vector<libpsaf_msgs::msg::StatusInfo> data);

/**
 * Helper Method to generate the libpsaf StatusInfo Messages
 * @param[in] values vector containing the value of the type parameter of the StatusInfo
 * @return the generated messages
 */
std::vector<libpsaf_msgs::msg::StatusInfo> generate_messages(std::vector<int> values)
{
  std::vector<libpsaf_msgs::msg::StatusInfo> result;
  for (int value : values) {
    libpsaf_msgs::msg::StatusInfo msg;
    msg.type = value;
    result.push_back(msg);
  }
  return result;
}

/**
 * Definition of multiple state transition sequences
 */
std::vector<int> full_run_park_parallel{0, 1, 2, 3, 9, 11, 13, 15, 16};
std::vector<int> full_run_park_perpendicular{0, 9, 12, 13, 15, 16};
std::vector<int> no_spot_found{0, 9, 10};
std::vector<int> parking_failed_parallel{0, 9, 11, 14, 16};
std::vector<int> parking_failed_perpendicular{0, 9, 12, 14, 16};
std::vector<int> all_state_discipline_one{0, 1, 2, 3, 9, 11, 13, 15, 16, 9, 12, 14};
std::vector<int> illegal_transitions_dr_normal{0, 0, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 24, 25};
std::vector<int> illegal_transitions_dr_normal_d_two{0, 2, 3, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15,
  16,
  17, 18, 19, 20, 21, 22, 24, 25};
std::vector<int> full_run_with_obstacles{0, 4, 5, 7, 6, 8, 17, 19, 20, 21, 8, 18, 19, 20, 21, 8,
  18, 22};
std::vector<int> all_state_discipline_two{0, 4, 5, 6, 8, 17, 19, 20, 21, 8, 19};
std::vector<int> leave_manual_mode{0, 23, 24};
std::vector<int> stays_in_manual_mode{0, 23, 15, 14};
std::vector<int> ignore_wrong_park{0, 9, 1};

std::vector<int> enter_shutdown{};

#endif  // STATE_MACHINE__INTEGRATION_TEST_HPP_
