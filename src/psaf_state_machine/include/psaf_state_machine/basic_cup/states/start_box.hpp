/**
 * @file start_box.hpp
 * @brief Definition of the start box state.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__START_BOX_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__START_BOX_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief The start box state.
 * This state is the start state of the state machine.
 * In this state, the car is located in the start box and waits for the startbox to open.
 */
class StartBox : public BasicCupStateMachine
{
public:
  StartBox() {stateID = STATE::STARTBOX;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
  void react(ButtonEvent const & e) override;
};

#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__START_BOX_HPP_
