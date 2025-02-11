/**
 * @file basic_cup_state_machine.hpp
 * @brief Definition of the basic cup state machine. Superclass to all state machines.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_HPP_

#include <functional>
#include "tinyfsm/tinyfsm.hpp"
#include "psaf_state_machine/basic_cup/basic_cup_state_machine_events.hpp"

/**
 * @class BasicCupStateMachine
 * @implements tinyfsm::Fsm
 * @brief The basic cup state machine.
 * @details This class is the basic cup state machine.
 */
class BasicCupStateMachine : public tinyfsm::Fsm<BasicCupStateMachine>
{
public:
  void react(tinyfsm::Event const & event)
  {
    (void) event;
  }

  virtual void entry();

  void exit() {}

  virtual void react(StatusEvent const & e);
  virtual void react(ButtonEvent const & e);
  virtual void react(ErrorEvent const & e);
  virtual void react(EnterStartState const & e);

public:
  int stateID;
  int discipline{-1};
};

#endif  // PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_HPP_
