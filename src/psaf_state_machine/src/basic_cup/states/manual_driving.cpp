/**
 * @file manual_driving.cpp
 * @brief the implementation of the manual driving state.
 * @author PSAF
 * @date 2022-06-01
 */
#include "tinyfsm/tinyfsm.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"
#include "psaf_state_machine/basic_cup/states/driving.hpp"
#include "psaf_state_machine/basic_cup/states/manual_driving.hpp"

void ManualDriving::entry()
{
}

void ManualDriving::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_EXIT) {
    transit<NormalDriving>();
  }
}
