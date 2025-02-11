/**
 * @file driving.cpp
 * @brief the implementation of the driving states.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/states/driving.hpp"
#include "psaf_state_machine/basic_cup/states/parking.hpp"
#include "psaf_state_machine/basic_cup/states/overtake.hpp"
#include "psaf_state_machine/basic_cup/states/manual_driving.hpp"
#include "psaf_state_machine/basic_cup/states/error.hpp"
#include "tinyfsm/tinyfsm.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include "libpsaf_msgs/msg/error.hpp"

void Driving::entry()
{
  EnterStartState e;
  dispatch<EnterStartState>(e);
}

void Driving::react(EnterStartState const & e)
{
  (void)e;
  transit<NormalDriving>();
}

void Driving::react(StatusEvent const & e)
{
  // Only enter in discipline 1
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::OBJECT_DETECTED && discipline == 1) {
    transit<Overtake>();
  }

  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER) {
    transit<ManualDriving>();
  }
}

void Driving::react(ErrorEvent const & e)
{
  if (e.error_message.type == libpsaf_msgs::msg::Error::ERROR) {
    transit<Error>();
  }
}

void Driving::react(ButtonEvent const & e)
{
  discipline = e.btn.data;
}

void NormalDriving::entry()
{
  BasicCupStateMachine::entry();
}

void NormalDriving::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::UPHILL_START) {
    transit<UphillDriving>();
  }

  if ((e.status_message.type == libpsaf_msgs::msg::StatusInfo::PARKING_INTENT) &&
    discipline == 0)
  {
    transit<Parking>();
  }

  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::STOP_LINE_APPROACH &&
    discipline == 1)
  {
    transit<ApproachIntersection>();
  }
  Driving::react(e);
}

void UphillDriving::entry()
{
  BasicCupStateMachine::entry();
}

void UphillDriving::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::DOWNHILL_START) {
    transit<DownhillDriving>();
  }
  Driving::react(e);
}

void DownhillDriving::entry()
{
  BasicCupStateMachine::entry();
}

void DownhillDriving::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::DOWNHILL_END) {
    transit<NormalDriving>();
  }
  Driving::react(e);
}

void WaitAtIntersection::entry()
{
  BasicCupStateMachine::entry();
}

void WaitAtIntersection::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::CONTINUE_NO_OBJECT) {
    transit<NormalDriving>();
  }
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::WAIT_FOR_OBJECT) {
    transit<WaitAtIntersection>();
  }
  Driving::react(e);
}

void ApproachIntersection::entry()
{
  BasicCupStateMachine::entry();
}

void ApproachIntersection::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::STOP_LINE_REACHED) {
    transit<WaitAtIntersection>();
  }

  Driving::react(e);
}
