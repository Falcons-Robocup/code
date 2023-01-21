// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_ROLES_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_ROLES_HPP_

// standard/system headers
#include <vector>
#include <string>
#include <map>

// other MTP headers
#include "PlayerId.hpp"

namespace mtp
{

// explicit numbering for compatibility reasons
enum class RoleEnum
{
    UNDEFINED = 0,
    GOALKEEPER = 1,
    ATTACKER_MAIN = 2,
    ATTACKER_ASSIST = 3,
    ATTACKER_GENERIC = 4,
    DEFENDER_MAIN = 5,
    DEFENDER_GENERIC = 6,
    DISABLED_OUT = 7,
    DISABLED_IN = 8
}; // end of enum class RoleEnum

struct PreferredRole
{
    RoleEnum role = RoleEnum::UNDEFINED;
    float factor = 1.0;
}; // end of struct PreferredRole

// conversion functions
std::string roleEnumToString(RoleEnum r);
RoleEnum roleStringToEnum(std::string s);
std::vector<RoleEnum> allAssignableRoles();

// checking against the min/max requirements
bool checkRoleCount(RoleEnum role, int count); // true == ok
typedef std::map<RoleEnum, int> RoleCount;
bool checkRoleCount(RoleCount const &roleCount); // true == ok
std::vector<RoleEnum> calculateAvailableRoles(RoleCount const &assignedRoleCount);
void getMinMaxRoleCount(RoleEnum role, int &minCount, int &maxCount);

// role allocation / assignment
typedef std::map<PlayerId, RoleEnum> RoleAllocation;

} // end of namespace mtp

#endif
