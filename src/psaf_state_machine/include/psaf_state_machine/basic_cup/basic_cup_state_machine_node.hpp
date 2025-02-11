/**
 * @file basic_cup_state_machine_node.hpp
 * @brief Definition of the basic cup state machine node. In here are the subscriber and publisher definitions.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_NODE_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_NODE_HPP_

#include "libpsaf/interface/state_machine_interface.hpp"
#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_configuration/configuration.hpp"

/**
 * @class BasicCupStateMachineNode
 * @implements StateMachineInterface
 * @brief The basic cup state machine node.
 * @details This class is the node of the state machine.
 * The current state of the state machine dictates the behavior of the car.
 * The state is passed to all other nodes. A transition is triggered by the status info topic.
 */
class BasicCupStateMachineNode : public libpsaf::StateMachineInterface
{
public:
  BasicCupStateMachineNode();

  void update();

protected:
  void processErrorMessage(libpsaf_msgs::msg::Error::SharedPtr p);

  void processStatus(libpsaf_msgs::msg::StatusInfo::SharedPtr s);

  void processSign(libpsaf_msgs::msg::Sign::SharedPtr sign) override;

  void processButton(std_msgs::msg::Int8::SharedPtr p);

private:
  BasicCupStateMachine machine;
};

#endif  // PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_NODE_HPP_
