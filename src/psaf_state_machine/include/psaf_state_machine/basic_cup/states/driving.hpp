/**
 * @file driving.hpp
 * @brief Definition of all driving subautomat. It contains all the substates
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__DRIVING_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__DRIVING_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief Driving state.
 * Main class for the driving sub-state machine.
 */
class Driving : public BasicCupStateMachine
{
protected:
  void entry() override;
  void react(EnterStartState const & e) override;
  void react(StatusEvent const & e) override;
  void react(ErrorEvent const & e) override;
  void react(ButtonEvent const & e) override;
  int discipline = -1;
};

/**
 * @brief Normal Driving state.
 * This state is entered when the robot is driving normally.
 * This means, no up-/downhill, no parking, no overtaking, no errors.
 */
class NormalDriving : public Driving
{
public:
  NormalDriving() {stateID = STATE::DR_NORMAL;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Uphill Driving state.
 * This state is entered when the robot is driving uphill.
 */
class UphillDriving : public Driving
{
public:
  UphillDriving() {stateID = STATE::DR_UP_HILL;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Downhill Driving state.
 * This state is entered when the robot is driving downhill.
 */
class DownhillDriving : public Driving
{
public:
  DownhillDriving() {stateID = STATE::DR_DOWN_HILL;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief The waiting state.
 * This state is entered when the robot is waiting at an intersection in discipline two.
 * The car needs to stop at the stop line in all cases. If an object with the right of way
 * is detected, the car has to wait until the object is gone.
 */
class WaitAtIntersection : public Driving
{
public:
  WaitAtIntersection() {stateID = STATE::DR_WAIT;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief The approaching state.
 * This state is entered when the robot is approaching an intersection in discipline two.
 */
class ApproachIntersection : public Driving
{
public:
  ApproachIntersection() {stateID = STATE::DR_APPROACH;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__DRIVING_HPP_
