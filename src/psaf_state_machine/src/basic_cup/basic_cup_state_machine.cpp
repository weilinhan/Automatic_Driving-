/**
 * @file basic_cup_state_machine.cpp
 * @brief the implementation of the basic cup state machine events.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/basic_cup/states/error.hpp"
#include "psaf_state_machine/basic_cup/states/start_box.hpp"


void BasicCupStateMachine::entry()
{
}

void BasicCupStateMachine::react(StatusEvent const & e)
{
  (void) e;
}

void BasicCupStateMachine::react(ButtonEvent const & e)
{
  (void) e;
}

void BasicCupStateMachine::react(ErrorEvent const & e)
{
  (void)e;
  transit<Error>();
}

void BasicCupStateMachine::react(EnterStartState const & e)
{
  (void) e;
}
