/**
 * @file error.cpp
 * @brief the implementation of the error state.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/states/error.hpp"
#include "psaf_state_machine/basic_cup/states/manual_driving.hpp"
#include "psaf_state_machine/basic_cup/states/shutdown.hpp"

void Error::entry()
{
}

void Error::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::WATCHDOG_TIMEOUT) {
    transit<Shutdown>();
  }
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER) {
    transit<ManualDriving>();
  }
}
