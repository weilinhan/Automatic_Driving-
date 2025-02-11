/**
 * @file start_box.cpp
 * @brief the implementation of the start box state.
 * @author PSAF
 * @date 2022-06-01
 */
#include "psaf_state_machine/basic_cup/states/start_box.hpp"
#include "psaf_state_machine/basic_cup/states/driving.hpp"
#include "psaf_state_machine/basic_cup/states/manual_driving.hpp"
#include "tinyfsm/tinyfsm.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"


void StartBox::entry()
{
}

void StartBox::react(StatusEvent const & e)
{
  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::STARTBOX_OPEN && discipline != -1) {
    transit<Driving>();
    ButtonEvent e;
    e.btn.data = discipline;
    dispatch<ButtonEvent>(e);
  }

  if (e.status_message.type == libpsaf_msgs::msg::StatusInfo::MANUAL_MODE_ENTER) {
    transit<ManualDriving>();
  }
}

void StartBox::react(ButtonEvent const & e)
{
  discipline = e.btn.data;
}
