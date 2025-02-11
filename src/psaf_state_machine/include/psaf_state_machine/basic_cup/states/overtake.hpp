/**
 * @file overtake.hpp
 * @brief Definition of the overtake subautomat. Contains all the overtaking related substates.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__OVERTAKE_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__OVERTAKE_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief the overtaking sub-state machine
 * This state machine controls the operations during the overtaking phase.
 */
class Overtake : public BasicCupStateMachine
{
protected:
  void entry() override;
  void react(EnterStartState const & e) override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief the check overtaking sub-state machine
 * In this state, the car checks if overtaking is necessary and possible.
 */
class CheckOvertake
  : public Overtake
{
public:
  CheckOvertake() {stateID = STATE::OT_CHECK;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};


/**
 * @brief the overtake follow sub-state machine
 * In this state, the car follows an object until the overtaking is possible.
 */
class FollowOvertake
  : public Overtake
{
public:
  FollowOvertake() {stateID = STATE::OT_FOLLOW;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief the initiating sub-state machine
 * Initiate the overtaking maneuver.
 */
class InitOvertake
  : public Overtake
{
public:
  InitOvertake() {stateID = STATE::OT_INIT;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};


/**
 * @brief the overtaking-perform sub-state machine
 * Perform the overtaking maneuver.
 */
class PerformOvertake
  : public Overtake
{
public:
  PerformOvertake() {stateID = STATE::OT_PERFORM;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief the overtaking-finish sub-state machine
 * Finish the overtaking maneuver.
 */
class FinishOvertake
  : public Overtake
{
public:
  FinishOvertake() {stateID = STATE::OT_FINISH;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};


#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__OVERTAKE_HPP_
