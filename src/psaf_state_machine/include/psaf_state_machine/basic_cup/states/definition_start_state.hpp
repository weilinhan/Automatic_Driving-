/**
 * @file definition_start_state.hpp
 * @brief Definition of the start state
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__DEFINITION_START_STATE_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__DEFINITION_START_STATE_HPP_

#include "tinyfsm/tinyfsm.hpp"

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/basic_cup/states/start_box.hpp"

FSM_INITIAL_STATE(BasicCupStateMachine, StartBox)


#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__DEFINITION_START_STATE_HPP_
