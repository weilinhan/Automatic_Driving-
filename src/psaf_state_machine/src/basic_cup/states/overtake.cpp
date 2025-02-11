/**
 * @file overtake.cpp
 * @brief the implementation of the overtake states.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/states/overtake.hpp"
#include "tinyfsm/tinyfsm.hpp"
#include "psaf_state_machine/basic_cup/states/driving.hpp"
#include "psaf_state_machine/basic_cup/states/manual_driving.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"

void Overtake::entry()
{
  EnterStartState e;
  dispatch<EnterStartState>(e);
}

void Overtake::react(EnterStartState const & e)
{
  (void)e;
  transit<CheckOvertake>();
}

void Overtake::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER) {
    transit<ManualDriving>();
  }
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::OVERTAKE_ABORT) {
    transit<Driving>();
  }
}

void CheckOvertake::entry()
{
  BasicCupStateMachine::entry();
}

void CheckOvertake::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::STATIC_OBSTACLE) {
    transit<InitOvertake>();
  }

  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::DYNAMIC_OBSTACLE) {
    transit<FollowOvertake>();
  }

  Overtake::react(e);
}

void FollowOvertake::entry()
{
  BasicCupStateMachine::entry();
}

void FollowOvertake::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE) {
    transit<PerformOvertake>();
  } else {
    Overtake::react(e);
  }
}

void PerformOvertake::entry()
{
  BasicCupStateMachine::entry();
}

void PerformOvertake::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::PASSED_OBSTACLE) {
    transit<FinishOvertake>();
  } else {
    Overtake::react(e);
  }
}

void FinishOvertake::entry()
{
  BasicCupStateMachine::entry();
}

void FinishOvertake::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::OVERTAKE_FINISHED) {
    transit<Driving>();
  } else {
    Overtake::react(e);
  }
}

void InitOvertake::entry()
{
  BasicCupStateMachine::entry();
}

void InitOvertake::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::OVERTAKE_POSSIBLE) {
    transit<PerformOvertake>();
  } else {
    Overtake::react(e);
  }
}
