/**
 * @file parking.cpp
 * @brief the implementation of the parking states.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/states/parking.hpp"
#include "psaf_state_machine/basic_cup/states/driving.hpp"
#include "psaf_state_machine/basic_cup/states/manual_driving.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include "tinyfsm/tinyfsm.hpp"

void Parking::entry()
{
  EnterStartState e;
  dispatch<EnterStartState>(e);
}

void Parking::react(EnterStartState const & e)
{
  (void)e;
  transit<SearchSpot>();
}

void Parking::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER) {
    transit<ManualDriving>();
  }
}

void SearchSpot::entry()
{
  BasicCupStateMachine::entry();
}

void SearchSpot::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARALLEL_FOUND) {
    transit<ParallelParking>();
  } else if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PERPENDICULAR_FOUND) {
    transit<PerpendicularParking>();
  } else if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARKING_TIMEOUT) {
    transit<Driving>();
  } else {
    Parking::react(e);
  }
}

void ParallelParking::entry()
{
  BasicCupStateMachine::entry();
}

void ParallelParking::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARKING_FINISHED) {
    transit<FinishParking>();
  } else if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARKING_FAILED) {
    transit<BackToLane>();
  } else {
    Parking::react(e);
  }
}

void PerpendicularParking::entry()
{
  BasicCupStateMachine::entry();
}

void PerpendicularParking::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARKING_FINISHED) {
    transit<FinishParking>();
  } else if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARKING_FAILED) {
    transit<BackToLane>();
  } else {
    Parking::react(e);
  }
}

void BackToLane::entry()
{
  BasicCupStateMachine::entry();
}

void BackToLane::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::BACK_ON_LANE) {
    transit<Driving>();
  } else {
    Parking::react(e);
  }
}

void FinishParking::entry()
{
  BasicCupStateMachine::entry();
}

void FinishParking::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARK_TIME_REACHED) {
    transit<BackToLane>();
  } else {
    Parking::react(e);
  }
}
