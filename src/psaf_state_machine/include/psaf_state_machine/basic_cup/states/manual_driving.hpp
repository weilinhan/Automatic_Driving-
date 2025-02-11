/**
 * @file manual_driving.hpp
 * @brief Definition of the manual driving state.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__MANUAL_DRIVING_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__MANUAL_DRIVING_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief The manual driving state.
 * This state is entered when the car is manually driven.
 * In this state, no other node should issue control commands.
 */
class ManualDriving : public BasicCupStateMachine
{
public:
  ManualDriving() {stateID = STATE::MANUAL_MODE;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};


#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__MANUAL_DRIVING_HPP_
