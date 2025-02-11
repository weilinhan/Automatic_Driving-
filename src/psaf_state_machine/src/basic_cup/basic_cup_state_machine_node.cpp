/**
 * @file basic_cup_state_machine_node.cpp
 * @brief the implementation of the basic cup state machine node.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/basic_cup_state_machine_node.hpp"
#include "psaf_state_machine/basic_cup/states/definition_start_state.hpp"

BasicCupStateMachineNode::BasicCupStateMachineNode()
: StateMachineInterface(
    STATE_MACHINE_NODE_NAME,
    WATCHDOG_ERROR_TOPIC,
    STATUS_INFO_TOPIC,
    BUTTON_TOPIC,
    SIGN_TOPIC,
    STATE_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
)
{
  machine.start();
}

void BasicCupStateMachineNode::processSign(libpsaf_msgs::msg::Sign::SharedPtr sign)
{
  int type = sign->type;
  // If a parking sign is detected.
  if (type == 3) {
    // Create a parking status info and publish it.
    StatusEvent eStatus;
    libpsaf_msgs::msg::StatusInfo status;
    status.type = 9;
    eStatus.status_message = status;
    machine.dispatch<StatusEvent>(eStatus);
    publishState(machine.current_state_ptr->stateID);
  }
}

void BasicCupStateMachineNode::processErrorMessage(libpsaf_msgs::msg::Error::SharedPtr p)
{
  if (p->type == 2) {
    ErrorEvent e;
    e.error_message = *p;
    machine.dispatch<ErrorEvent>(e);
  }
  publishState(machine.current_state_ptr->stateID);
}

void BasicCupStateMachineNode::processStatus(libpsaf_msgs::msg::StatusInfo::SharedPtr s)
{
  StatusEvent e;
  e.status_message = *s;
  machine.dispatch<StatusEvent>(e);
  publishState(machine.current_state_ptr->stateID);
}

void BasicCupStateMachineNode::processButton(std_msgs::msg::Int8::SharedPtr p)
{
  ButtonEvent e;
  e.btn = *p;
  machine.dispatch<ButtonEvent>(e);
}

void BasicCupStateMachineNode::update()
{
  publishState(machine.current_state_ptr->stateID);
}
