/**
 * @file shutdown.hpp
 * @brief Definition of the shutdown state. This state is final and cannot be exited.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__SHUTDOWN_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__SHUTDOWN_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief Shutdown state.
 * In this state, the car is in a safe state and can be safely shut down.
 * This state is only being entered, after a critical error has been detected
 * and the car was not able to recover from it.
 */
class Shutdown : public BasicCupStateMachine
{
public:
  Shutdown() {stateID = STATE::SHUTDOWN;}

protected:
  void entry() override;
};

#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__SHUTDOWN_HPP_
