/**
 * @file parking.hpp
 * @brief Definition of the parking subautomat. It contains all the substates for the parking phase.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__STATES__PARKING_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__STATES__PARKING_HPP_

#include "psaf_state_machine/basic_cup/basic_cup_state_machine.hpp"
#include "psaf_state_machine/state_definitions.hpp"

/**
 * @brief Parking state
 * This subautomat handles the parking maneuver.
 */
class Parking : public BasicCupStateMachine
{
protected:
  void entry() override;

  void react(EnterStartState const & e) override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Search State
 * In this state the car searches for a parking spot
 */
class SearchSpot : public Parking
{
public:
  SearchSpot() {stateID = STATE::PR_SEARCH;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Parallel Parking State
 * In this state the car parks parallel in the parking spot
 */
class ParallelParking : public Parking
{
public:
  ParallelParking() {stateID = STATE::PR_PARALLEL;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Perpendicular Parking State
 * In this state the car parks perpendicular in the parking spot
 */
class PerpendicularParking : public Parking
{
public:
  PerpendicularParking() {stateID = STATE::PR_PERPENDICULAR;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Back to Lane State
 * In this state the car returns to the lane after parking
 */
class BackToLane : public Parking
{
public:
  BackToLane() {stateID = STATE::PR_RETURN;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};

/**
 * @brief Parking Finished State
 * In this state the car is parked and waits for the required time to pass.
 */
class FinishParking : public Parking
{
public:
  FinishParking() {stateID = STATE::PR_FINISH;}

protected:
  void entry() override;
  void react(StatusEvent const & e) override;
};


#endif  // PSAF_STATE_MACHINE__BASIC_CUP__STATES__PARKING_HPP_
