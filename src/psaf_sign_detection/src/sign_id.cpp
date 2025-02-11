/**
 * @file sign_id.cpp
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#include "psaf_sign_detection/sign_id.hpp"

#include <sstream>

sign_id::SignId_type sign_id::id2underlying(const SignId id)
{
  return static_cast<SignId_type>(id);
}

sign_id::SignId sign_id::underlying2id(const SignId_type underlying)
{
  return static_cast<SignId>(underlying);
}

std::string sign_id::getSignName(const SignId id)
{
  if (signIdNameMap.count(id) != 0) {
    return signIdNameMap.at(id);
  } else {
    std::stringstream ss;
    ss << "sign id " << sign_id::id2underlying(id) << " is not in signIdNameMap!";
    throw std::runtime_error(ss.str());
  }
}

void sign_id::verifyModel(const std::vector<std::string> classes)
{
  // first we check all three data structures have the same count
  if (sign_id::SignIdCount != classes.size()) {
    throw std::runtime_error(
            "Error verifying model: Number of SignId entries dont match classes.size()!");
  }

  if (sign_id::SignIdCount != signIdNameMap.size()) {
    throw std::runtime_error(
            "Error verifying model: Number of SignId entries dont match signIdNameMap.size()!");
  }

  if (signIdNameMap.size() != classes.size()) {
    throw std::runtime_error(
            "Error verifying model: signIdNameMap.size() doesnt match classes.size()!");
  }

  // then we check class names and our defined names match
  for (unsigned int i = 0; i < classes.size(); ++i) {
    if (classes[i] != getSignName(underlying2id(i))) {
      throw std::runtime_error("Error verifying model: got different names for same id!");
    }
  }
}

int sign_id::signIdToPsafId(const SignId id)
{
  int val;
  switch (id) {
    case SignId::PedestrianCrossing:
      val = libpsaf_msgs::msg::Sign::CROSSWALK;
      break;
    case SignId::Priority:
      val = libpsaf_msgs::msg::Sign::PRIORITY;
      break;
    case SignId::SpeedLimit30:
      // NOTE: There is no identifier for this in libpsaf_msgs::msg::Sign ...
      // so we use another one
      val = libpsaf_msgs::msg::Sign::UPHILL;
      break;
    case SignId::Stop:
      val = libpsaf_msgs::msg::Sign::STOP;
      break;
    case SignId::Yield:
      val = libpsaf_msgs::msg::Sign::YIELD;
      break;
    case SignId::Zone30Start:
      val = libpsaf_msgs::msg::Sign::LIMIT_30_START;
      break;
    case SignId::Zone30End:
      val = libpsaf_msgs::msg::Sign::LIMIT_30_END;
      break;
    default:
      throw std::runtime_error("Unhandled case in signIdToPsafId!");
  }
  return val;
}
