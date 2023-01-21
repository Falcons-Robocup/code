// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "ext/Roles.hpp"

// standard/system headers
#include <boost/bimap.hpp>
#include <boost/assign.hpp>


using namespace mtp;


// enum/string conversions

typedef boost::bimap<RoleEnum, std::string> bm_type;
const bm_type roleEnumToStringBiMap = 
  boost::assign::list_of<bm_type::relation>
    (RoleEnum::UNDEFINED, std::string("UNDEFINED"))
    (RoleEnum::GOALKEEPER, std::string("GOALKEEPER"))
    (RoleEnum::ATTACKER_MAIN, std::string("ATTACKER_MAIN"))
    (RoleEnum::ATTACKER_ASSIST, std::string("ATTACKER_ASSIST"))
    (RoleEnum::ATTACKER_GENERIC, std::string("ATTACKER_GENERIC"))
    (RoleEnum::DEFENDER_MAIN, std::string("DEFENDER_MAIN"))
    (RoleEnum::DEFENDER_GENERIC, std::string("DEFENDER_GENERIC"))
    (RoleEnum::DISABLED_OUT, std::string("DISABLED_OUT"))
    (RoleEnum::DISABLED_IN, std::string("DISABLED_IN"));

std::string mtp::roleEnumToString(RoleEnum r)
{
    return roleEnumToStringBiMap.left.at(r);
}

RoleEnum mtp::roleStringToEnum(std::string s)
{
    return roleEnumToStringBiMap.right.at(s);
}

std::vector<RoleEnum> mtp::allAssignableRoles()
{
    std::vector<RoleEnum> result = {
        RoleEnum::GOALKEEPER,
        RoleEnum::ATTACKER_MAIN,
        RoleEnum::ATTACKER_ASSIST,
        RoleEnum::ATTACKER_GENERIC,
        RoleEnum::DEFENDER_MAIN,
        RoleEnum::DEFENDER_GENERIC,
        RoleEnum::DISABLED_OUT,
        RoleEnum::DISABLED_IN
    };
    return result;
}

// role count rule specifications

const std::map<RoleEnum, int> minimumRoleCount = {
    {RoleEnum::UNDEFINED, 0},
    {RoleEnum::GOALKEEPER, 1},
    {RoleEnum::ATTACKER_MAIN, 1},
    {RoleEnum::ATTACKER_ASSIST, 0},
    {RoleEnum::ATTACKER_GENERIC, 0},
    {RoleEnum::DEFENDER_MAIN, 1}
};

const std::map<RoleEnum, int> maximumRoleCount = {
    {RoleEnum::UNDEFINED, 0},
    {RoleEnum::GOALKEEPER, 1},
    {RoleEnum::ATTACKER_MAIN, 1},
    {RoleEnum::ATTACKER_ASSIST, 1},
    {RoleEnum::ATTACKER_GENERIC, 0}, // disable for now, Falcons teamplay / trees need more work to fully support this new role
    {RoleEnum::DEFENDER_MAIN, 1}
};

void mtp::getMinMaxRoleCount(RoleEnum role, int &minCount, int &maxCount)
{
    if (minimumRoleCount.count(role)) minCount = minimumRoleCount.at(role);
    else minCount = 0;
    if (maximumRoleCount.count(role)) maxCount = maximumRoleCount.at(role);
    else maxCount = 11;
}

// role count checkers and convenience functions

#include "tprintf.hpp" // DEBUG

bool mtp::checkRoleCount(RoleEnum role, int count)
{
    bool result = true;
    // check minimum, if specified
    if (minimumRoleCount.count(role))
    {
        if (count < minimumRoleCount.at(role)) result = false;
    }
    // check maximum, if specified
    if (maximumRoleCount.count(role))
    {
        if (count > maximumRoleCount.at(role)) result = false;
    }
    // done
    //tprintf("debug: role=%s count=%d result=%d", roleEnumToString(role).c_str(), count, result);
    return result;
}

bool mtp::checkRoleCount(std::map<RoleEnum, int> const &roleCount)
{
    for (auto rp: roleEnumToStringBiMap.left)
    {
        auto role = rp.first;
        int count = 0;
        if (roleCount.count(role)) count = roleCount.at(role);
        if (!checkRoleCount(role, count)) return false;
    }
    return true;
}

std::vector<RoleEnum> mtp::calculateAvailableRoles(std::map<RoleEnum, int> const &assignedRoleCount)
{
    // determine playable roles
    // TODO: make available in Roles.hpp?
    std::vector<RoleEnum> playableRoles = {
        RoleEnum::GOALKEEPER,
        RoleEnum::ATTACKER_MAIN,
        RoleEnum::ATTACKER_ASSIST,
        //RoleEnum::ATTACKER_GENERIC,
        RoleEnum::DEFENDER_MAIN,
        RoleEnum::DEFENDER_GENERIC
        };
    // check available roles
    std::vector<RoleEnum> result; // actually a set...
    for (auto &role: playableRoles)
    {
        int count = 0;
        if (assignedRoleCount.count(role)) count = assignedRoleCount.at(role);
        // check if it would be fine to assign this role
        if (checkRoleCount(role, 1 + count))
        {
            result.push_back(role);
        }
    }
    return result;
}
