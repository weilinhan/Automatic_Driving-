/**
 * @file error.hpp
 * @brief Definition of the error state
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__ERROR_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__ERROR_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief Error state.
 * This state is entered when a critical error occurs. (e.g. camera image is not available)
 */
class Error : public BasicCupStateMachine
{
public:
  Error() {stateID = STATE::ERROR;}

protected:
  void entry() override;

  void react(StatusEvent const & e) override;
};


#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__ERROR_HPP_
