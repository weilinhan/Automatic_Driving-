/**
 * @file test_state_machine_discipline_one.cpp
 * @brief Test for the states in discipline one.
 * @author PSAF
 * @date 2022-06-01
 */
#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "gtest/gtest.h"
#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"
#include "psaf_state_machine/basic_cup/states/definition_start_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "libpsaf_msgs/msg/status_info.hpp"
#include "libpsaf_msgs/msg/error.hpp"

std::vector<std::vector<int>> getValidTransitions();

std::vector<std::vector<int>> val = getValidTransitions();

/**
* @class StateMachineTests
* @brief Test suite for the states in discipline one.
*/

class TestTransitionsForDisciplineOne : public testing::TestWithParam<std::vector<int>>
{
protected:
  BasicCupStateMachine state_machine;
};

/**
 * Parameterized test case. Will receive an vector contains initial state, transition and target state
 */
TEST_P(TestTransitionsForDisciplineOne, TestIfValidTransitionsWorks)
{
  StatusEvent event;
  libpsaf_msgs::msg::StatusInfo status;
  int start_state = GetParam().at(0);
  int msg_type = GetParam().at(1);
  int target_state = GetParam().at(2);
  /**
   * Prevent constant reinitialization. This would cause the tests to fail
   */
  if (start_state == 0) {
    state_machine.start();
    ButtonEvent eButton;
    eButton.btn.data = 0;
    state_machine.dispatch<ButtonEvent>(eButton);
  }

  status.type = msg_type;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, target_state) <<
    "Transition from " << start_state << " to " << target_state << " failed";
}

/**
 * Generator for the test cases
 */
INSTANTIATE_TEST_CASE_P(
  TransitionTests,
  TestTransitionsForDisciplineOne,
  ::testing::ValuesIn(val));

/**
 * Check if the state machine reacts to a invalid overtake transition
 */
TEST_F(TestTransitionsForDisciplineOne, TestIfStateMachineDoesNotReactToInvalidOvertakeTransition)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 0;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent event;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  StatusEvent event2;
  libpsaf_msgs::msg::StatusInfo status2;
  status2.type = libpsaf_msgs::msg::StatusInfo::Type::OVERTAKE_POSSIBLE;
  event2.status_message = status2;
  state_machine.dispatch<StatusEvent>(event2);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::DR_NORMAL) <<
    "Car reacted to invalid overtake transition";
}

/**
 * Check if the state machine reacts to a invalid independent transition
 */
TEST_F(TestTransitionsForDisciplineOne, TestDoesNotReactToInvalidIndependentTransition)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 0;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent event;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  StatusEvent event2;
  libpsaf_msgs::msg::StatusInfo status2;
  status2.type = libpsaf_msgs::msg::StatusInfo::Type::WATCHDOG_TIMEOUT;
  event2.status_message = status2;
  state_machine.dispatch<StatusEvent>(event2);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::DR_NORMAL) <<
    "Car reacted to invalid independent transition";
}

/**
 * Test if the state machine can transition into error state from the driving subautomat
 */
TEST_F(TestTransitionsForDisciplineOne, TestCanEnterErrorModeInDriving)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 0;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent event;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  ErrorEvent eError;
  libpsaf_msgs::msg::Error error;
  error.type = libpsaf_msgs::msg::Error::ERROR;
  eError.error_message = error;
  state_machine.dispatch<ErrorEvent>(eError);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::ERROR) <<
    "Car did not enter error state from driving";
}

/**
 * Test if the state machine can enter the error state from the parking subautomat
 */
TEST_F(TestTransitionsForDisciplineOne, TestCanEnterErrorFromParking)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 0;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent event;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::Type::STARTBOX_OPEN;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  StatusEvent event2;
  libpsaf_msgs::msg::StatusInfo status2;
  status2.type = libpsaf_msgs::msg::StatusInfo::Type::PARKING_INTENT;
  event2.status_message = status2;
  state_machine.dispatch<StatusEvent>(event2);
  ErrorEvent eError;
  libpsaf_msgs::msg::Error error;
  error.type = libpsaf_msgs::msg::Error::ERROR;
  eError.error_message = error;
  state_machine.dispatch<ErrorEvent>(eError);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::ERROR) <<
    "Car did not enter error state from parking";
}


/**
 * Helper method to create all valid transitions. DO NOT CHANGE THE ORDER OF THIS VECTORS!!
 * @return a vector of vectors containing the transitions
 */
std::vector<std::vector<int>> getValidTransitions(void)
{
  std::vector<std::vector<int>> values{
    {STATE::STARTBOX, libpsaf_msgs::msg::StatusInfo::STARTBOX_OPEN, STATE::DR_NORMAL},
    // Start uphill and downhill with manual aborts
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::UPHILL_START, STATE::DR_UP_HILL},
    {STATE::DR_UP_HILL, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::UPHILL_START, STATE::DR_UP_HILL},
    {STATE::DR_UP_HILL, libpsaf_msgs::msg::StatusInfo::DOWNHILL_START, STATE::DR_DOWN_HILL},
    {STATE::DR_UP_HILL, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::UPHILL_START, STATE::DR_UP_HILL},
    {STATE::DR_UP_HILL, libpsaf_msgs::msg::StatusInfo::DOWNHILL_START, STATE::DR_DOWN_HILL},
    {STATE::DR_DOWN_HILL, libpsaf_msgs::msg::StatusInfo::DOWNHILL_END, STATE::DR_NORMAL},
    // Normal parallel parking
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::PARALLEL_FOUND, STATE::PR_PARALLEL},
    {STATE::PR_PARALLEL, libpsaf_msgs::msg::StatusInfo::PARKING_FINISHED, STATE::PR_FINISH},
    {STATE::PR_FINISH, libpsaf_msgs::msg::StatusInfo::PARK_TIME_REACHED, STATE::PR_RETURN},
    {STATE::PR_RETURN, libpsaf_msgs::msg::StatusInfo::BACK_ON_LANE, STATE::DR_NORMAL},
    // Normal Perpendicular Parking
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::PERPENDICULAR_FOUND, STATE::PR_PERPENDICULAR},
    {STATE::PR_PERPENDICULAR, libpsaf_msgs::msg::StatusInfo::PARKING_FINISHED, STATE::PR_FINISH},
    {STATE::PR_FINISH, libpsaf_msgs::msg::StatusInfo::PARK_TIME_REACHED, STATE::PR_RETURN},
    {STATE::PR_RETURN, libpsaf_msgs::msg::StatusInfo::BACK_ON_LANE, STATE::DR_NORMAL},
    // Perpendicular with manual abort
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::PERPENDICULAR_FOUND, STATE::PR_PERPENDICULAR},
    {STATE::PR_PERPENDICULAR, libpsaf_msgs::msg::StatusInfo::PARKING_FINISHED, STATE::PR_FINISH},
    {STATE::PR_FINISH, libpsaf_msgs::msg::StatusInfo::PARK_TIME_REACHED, STATE::PR_RETURN},
    {STATE::PR_RETURN, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Search abort by manual mode
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Parking search timeout
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::PARKING_TIMEOUT, STATE::DR_NORMAL},
    // Failed parallel park
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::PARALLEL_FOUND, STATE::PR_PARALLEL},
    {STATE::PR_PARALLEL, libpsaf_msgs::msg::StatusInfo::PARKING_FAILED, STATE::PR_RETURN},
    {STATE::PR_RETURN, libpsaf_msgs::msg::StatusInfo::BACK_ON_LANE, STATE::DR_NORMAL},
    // Failed perpendicular park
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::PARKING_INTENT, STATE::PR_SEARCH},
    {STATE::PR_SEARCH, libpsaf_msgs::msg::StatusInfo::PERPENDICULAR_FOUND, STATE::PR_PERPENDICULAR},
    {STATE::PR_PERPENDICULAR, libpsaf_msgs::msg::StatusInfo::PARKING_FAILED, STATE::PR_RETURN},
    {STATE::PR_RETURN, libpsaf_msgs::msg::StatusInfo::BACK_ON_LANE, STATE::DR_NORMAL}
  };
  return values;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
