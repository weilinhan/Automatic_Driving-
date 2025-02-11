/**
 * @file test_state_machine_discipline_two.cpp
 * @brief Test for the states in discipline 2.
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
 * @class TestTransitionsForDisciplineTwo
 * @brief Test suite for the states in discipline two.
 */
class TestTransitionsForDisciplineTwo : public testing::TestWithParam<std::vector<int>>
{
protected:
  BasicCupStateMachine state_machine;
};

/**
 * Parameterized test case. Will receive an vector contains initial state, transition and target state
 */
TEST_P(TestTransitionsForDisciplineTwo, TestIfValidTransitionsWorks)
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
    eButton.btn.data = 1;
    state_machine.dispatch<ButtonEvent>(eButton);
  }

  status.type = msg_type;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, target_state);
}

/**
 * Generator for the test cases
 */
INSTANTIATE_TEST_CASE_P(
  TransitionTests,
  TestTransitionsForDisciplineTwo,
  ::testing::ValuesIn(val));

/**
 * Test if the state machine ignores invalid transitions from the parking subautomat
 */
TEST_F(TestTransitionsForDisciplineTwo, TestIgnoresInvalidParkingTransition)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 1;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent event;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::STARTBOX_OPEN;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  status.type = libpsaf_msgs::msg::StatusInfo::PARKING_INTENT;
  event.status_message = status;
  state_machine.dispatch<StatusEvent>(event);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::DR_NORMAL);
}

/**
 * Test if the state machine ignores invalid transitions
 */
TEST_F(TestTransitionsForDisciplineTwo, TestIgnoresInvalidIndependentTransition)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 1;
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
TEST_F(TestTransitionsForDisciplineTwo, TestCanEnterErrorModeInDriving)
{
  state_machine.reset();
  state_machine.start();
  ButtonEvent eButton;
  eButton.btn.data = 1;
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
TEST_F(TestTransitionsForDisciplineTwo, TestCanEnterErrorFromOvertake)
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
  status2.type = libpsaf_msgs::msg::StatusInfo::Type::OBJECT_DETECTED;
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
    {STATE::DR_UP_HILL, libpsaf_msgs::msg::StatusInfo::DOWNHILL_START, STATE::DR_DOWN_HILL},
    {STATE::DR_DOWN_HILL, libpsaf_msgs::msg::StatusInfo::DOWNHILL_END, STATE::DR_NORMAL},
    // Normal overtaking
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    // Static Obstacle
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::STATIC_OBSTACLE, STATE::OT_INIT},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE, STATE::OT_PERFORM},
    {STATE::OT_PERFORM, libpsaf_msgs::msg::StatusInfo::PASSED_OBSTACLE, STATE::OT_FINISH},
    {STATE::OT_FINISH, libpsaf_msgs::msg::StatusInfo::OVERTAKE_FINISHED, STATE::DR_NORMAL},
    // Normal overtaking
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    // Dynamic Obstacle
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE, STATE::OT_FOLLOW},
    {STATE::OT_FOLLOW, libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE, STATE::OT_PERFORM},
    {STATE::OT_PERFORM, libpsaf_msgs::msg::StatusInfo::PASSED_OBSTACLE, STATE::OT_FINISH},
    {STATE::OT_FINISH, libpsaf_msgs::msg::StatusInfo::OVERTAKE_FINISHED, STATE::DR_NORMAL},
    // check overtake abort in each state for static
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT, STATE::DR_NORMAL},
    // Till ot_init
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::STATIC_OBSTACLE, STATE::OT_INIT},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT, STATE::DR_NORMAL},
    // Till ot_perform
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::STATIC_OBSTACLE, STATE::OT_INIT},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE, STATE::OT_PERFORM},
    {STATE::OT_PERFORM, libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT, STATE::DR_NORMAL},
    // check overtake abort in each state for dynamic
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT, STATE::DR_NORMAL},
    // Till ot_init
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE, STATE::OT_FOLLOW},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT, STATE::DR_NORMAL},
    // Till ot_perform
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE, STATE::OT_FOLLOW},
    {STATE::OT_FOLLOW, libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE, STATE::OT_PERFORM},
    {STATE::OT_PERFORM, libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT, STATE::DR_NORMAL},
    // check manual mode for static
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Till ot_init
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::STATIC_OBSTACLE, STATE::OT_INIT},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Till ot_perform
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::STATIC_OBSTACLE, STATE::OT_INIT},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE, STATE::OT_PERFORM},
    {STATE::OT_PERFORM, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // check overtake abort in each state
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Till ot_init
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE, STATE::OT_FOLLOW},
    {STATE::OT_INIT, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Till ot_perform
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED, STATE::OT_CHECK},
    {STATE::OT_CHECK, libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE, STATE::OT_FOLLOW},
    {STATE::OT_FOLLOW, libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE, STATE::OT_PERFORM},
    {STATE::OT_PERFORM, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    // Check Stop Line Behaviour
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::STOP_LINE_APPROACH, STATE::DR_APPROACH},
    {STATE::DR_APPROACH, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::STOP_LINE_APPROACH, STATE::DR_APPROACH},
    {STATE::DR_APPROACH, libpsaf_msgs::msg::StatusInfo::STOP_LINE_REACHED, STATE::DR_WAIT},
    {STATE::DR_WAIT, libpsaf_msgs::msg::StatusInfo::WAIT_FOR_OBJECT, STATE::DR_WAIT},
    {STATE::DR_WAIT, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER, STATE::MANUAL_MODE},
    {STATE::MANUAL_MODE, libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT, STATE::DR_NORMAL},
    {STATE::DR_NORMAL, libpsaf_msgs::msg::StatusInfo::STOP_LINE_APPROACH, STATE::DR_APPROACH},
    {STATE::DR_APPROACH, libpsaf_msgs::msg::StatusInfo::STOP_LINE_REACHED, STATE::DR_WAIT},
    {STATE::DR_WAIT, libpsaf_msgs::msg::StatusInfo::CONTINUE_NO_OBJECT, STATE::DR_NORMAL}
  };
  return values;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
