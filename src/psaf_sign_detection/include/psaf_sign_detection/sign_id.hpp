/**
 * @file sign_id.hpp
 * @brief The definition of Sign IDs and helper functions
 * @author sven.kalmbach
 * @date 2024-02-08
 */
#ifndef PSAF_SIGN_DETECTION__SIGNID_HPP_
#define PSAF_SIGN_DETECTION__SIGNID_HPP_

#include <map>
#include <string>
#include <vector>
#include <initializer_list>

#include "libpsaf_msgs/msg/sign.hpp"

namespace sign_id
{

/**
 * @brief Underlying type for SignIds
 * @details is int, specified by interface of publishSign
 */
using SignId_type = int;


/**
 * @brief Number of entries in SignId, used for verifiyng mapping to classes.txt
 */
constexpr unsigned int SignIdCount = 7;

/**
 * @brief Enum used to encode sign
 * @details These need to map declaration order of classes.txt,
 *          since they are used as class ids.
 * @note Also update signIdToPsafId if you update this enum.
 */

enum class SignId : SignId_type
{
  PedestrianCrossing = 0,
  Priority = 1,
  SpeedLimit30 = 2,
  Stop = 3,
  Yield = 4,
  Zone30Start = 5,
  Zone30End = 6,
};

/**
 * @brief intializer list to be able to iterate over enum
 */
constexpr std::initializer_list<SignId> allSignId = {
  SignId::PedestrianCrossing,
  SignId::Priority,
  SignId::SpeedLimit30,
  SignId::Stop,
  SignId::Yield,
  SignId::Zone30Start,
  SignId::Zone30End,
};

/**
 * @brief Map containg human readable names for enum values
 */
static const std::map<SignId, std::string> signIdNameMap = {
  {SignId::PedestrianCrossing, "PedestrianCrossing"},
  {SignId::Priority, "Priority"},
  {SignId::SpeedLimit30, "SpeedLimit30"},
  {SignId::Stop, "Stop"},
  {SignId::Yield, "Yield"},
  {SignId::Zone30Start, "Zone30Start"},
  {SignId::Zone30End, "Zone30End"}
};

/**
 * @brief Get the SignId_type underyling as SignId
 * @param id the id to get the underyling type of
 * @return sign_typ the underlying value
 */
SignId_type id2underlying(const SignId id);

/**
 * @brief Get the SignId corresponding to underlying type
 * @param underlying the underlying value
 * @return SignId the corresponding SignId
 */
SignId underlying2id(const SignId_type underlying);

/**
 * @brief Get the name of a sign identifed by its it
 * @throws std::runtime_error if sign is not found in signIdNameMap
 * @param id the sign id to get the string name of
 * @return std::string the string name of the sign id
 */
std::string getSignName(const SignId id);

/**
 * @brief Verify the classes vector matches SignId enum and map
 * @param classes the classes vector to check
 */
void verifyModel(const std::vector<std::string> classes);

/**
 * @brief Convert a SignId to a libpsaf_msgs::msg::Sign::Type value
 * @note SignID::SpeedLimit30 is converted to libpsaf_msgs::msg::Sign::UPHILL
 * since there is no matching libpsaf_msgs::msg::Sign value...
 * @return int the libpsaf_msgs::msg::Sign::Type value
 */
int signIdToPsafId(const SignId id);

}  // namespace sign_id

#endif  // PSAF_SIGN_DETECTION__SIGNID_HPP_
