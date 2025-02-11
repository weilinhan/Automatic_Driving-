/**
 * @file test_state_machine.cpp
 * @brief Test for the independent states.
 * @author PSAF
 * @date 2022-06-01
 */
#include <iostream>
#include "gtest/gtest.h"

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"
#include "psaf_state_machine/basic_cup/states/definition_start_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "libpsaf_msgs/msg/status_info.hpp"
#include "libpsaf_msgs/msg/error.hpp"

/**
* @class StateMachineTests
 * @brief Test suite for the independent states.
 */
class StateMachineTests : public ::testing::Test
{
public:
  /**
   * @brief Setup for each test case.
   */
  void SetUp() override
  {
    state_machine.start();
  }

  /**
   * @brief Teardown for each test case.
   */
  void TearDown() override
  {
    state_machine.reset();
  }

protected:
  BasicCupStateMachine state_machine;
};


/**
 * Test if state_machine is in the Startbox State after initialization
 */
TEST_F(StateMachineTests, TestIsInStartboxState)
{
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}

/**
 * Test if the car stays in the startbox if no button was pressed
 */
TEST_F(StateMachineTests, TestStaysInStartBoxIfNoButtonWasPressedAndInvalidStatusInfo)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::UPHILL_START;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}


/**
 * Test if the car stays in the startbox if no button was pressed
 * true && false
 */
TEST_F(StateMachineTests, TestStaysInStartBoxIfNoButtonWasPressedButValidStatus)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::STARTBOX_OPEN;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}

/**
 * Test if the car stays in the startbox if no button was pressed
 * false && true
 */
TEST_F(StateMachineTests, TestStaysInStartBoxStateIfButtonWasPressedButInvalidStatusInfo)
{
  ButtonEvent eButton;
  eButton.btn.data = 0;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::PARKING_INTENT;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}

/**
 * Test if state_machine transitions into Normal_driving after StartBox opens
 * true && true
 */
TEST_F(StateMachineTests, TestTransitToNormalDrive)
{
  ButtonEvent eButton;
  eButton.btn.data = 0;
  state_machine.dispatch<ButtonEvent>(eButton);
  StatusEvent e;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::STARTBOX_OPEN;
  e.status_message = status;
  state_machine.dispatch<StatusEvent>(e);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::DR_NORMAL);
}

/**
 * Check if State Machine transitions into error state after error event
 */
TEST_F(StateMachineTests, TestTransitIntoErrorState)
{
  ErrorEvent eError;
  libpsaf_msgs::msg::Error error;
  error.type = libpsaf_msgs::msg::Error::ERROR;
  eError.error_message = error;
  state_machine.dispatch<ErrorEvent>(eError);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::ERROR);
}

/**
 * Check if the State Machine can transition into the Manual Mode state
 */
TEST_F(StateMachineTests, TestTransitToManualMode)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::MANUAL_MODE);
}

/**
 * Check if the State Machine return to DR_Normal if Manual Mode is exited
 */
TEST_F(StateMachineTests, TestCanReturnFromManualMode)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::MANUAL_MODE);
  StatusEvent eStatus2;
  libpsaf_msgs::msg::StatusInfo status2;
  status2.type = libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT;
  eStatus2.status_message = status2;
  state_machine.dispatch<StatusEvent>(eStatus2);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::DR_NORMAL);
}

/**
 * Test if manual mode can be reached from the error state
 */
TEST_F(StateMachineTests, TestCanEnterManualModeFromError)
{
  ErrorEvent eError;
  libpsaf_msgs::msg::Error error;
  error.type = libpsaf_msgs::msg::Error::ERROR;
  eError.error_message = error;
  state_machine.dispatch<ErrorEvent>(eError);
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::MANUAL_MODE);
}

/**
 * Go into shutdown after watchdog timeout event, indicating no recovery possible
 */
TEST_F(StateMachineTests, TestDoesNotRecoverInErrorState)
{
  ErrorEvent eError;
  libpsaf_msgs::msg::Error error;
  error.type = libpsaf_msgs::msg::Error::ERROR;
  eError.error_message = error;
  state_machine.dispatch<ErrorEvent>(eError);
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::WATCHDOG_TIMEOUT;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::SHUTDOWN);
}

/**
 * Test if invalid status from driving subautomat is handled
 */
TEST_F(StateMachineTests, TestDoesNotReactToStatusEventFromDriving)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::UPHILL_START;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}

/**
 * Test if invalid status from overtake subautomat is handled
 */
TEST_F(StateMachineTests, TestDoesNotReactToStatusEventFromOvertake)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}

/**
 * Test if invalid status from parking subautomat is handled
 */
TEST_F(StateMachineTests, TestDoesNotReactToStatusEventFromParking)
{
  StatusEvent eStatus;
  libpsaf_msgs::msg::StatusInfo status;
  status.type = libpsaf_msgs::msg::StatusInfo::PARALLEL_FOUND;
  eStatus.status_message = status;
  state_machine.dispatch<StatusEvent>(eStatus);
  ASSERT_EQ(state_machine.current_state_ptr->stateID, STATE::STARTBOX);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
