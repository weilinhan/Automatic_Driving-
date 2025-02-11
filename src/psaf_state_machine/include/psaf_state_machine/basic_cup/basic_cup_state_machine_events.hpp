/**
 * @file basic_cup_state_machine_events.hpp
 * @brief Definition of the events used by the basic cup state machine.
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_EVENTS_HPP_
#define PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_EVENTS_HPP_

#include <string>
#include "tinyfsm/tinyfsm.hpp"
#include "libpsaf_msgs/msg/error.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "libpsaf_msgs/msg/status_info.hpp"


struct ErrorEvent : tinyfsm::Event
{
  libpsaf_msgs::msg::Error error_message;
};

struct StatusEvent : tinyfsm::Event
{
  libpsaf_msgs::msg::StatusInfo status_message;
};

struct EnterStartState : tinyfsm::Event {};

struct ButtonEvent : tinyfsm::Event
{
  std_msgs::msg::Int8 btn;
};

#endif  // PSAF_STATE_MACHINE__BASIC_CUP__BASIC_CUP_STATE_MACHINE_EVENTS_HPP_
