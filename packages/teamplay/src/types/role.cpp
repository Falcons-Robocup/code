// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * role.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <string>

#include "int/types/role.hpp"
#include "cDiagnostics.hpp"

using namespace teamplay;


role::role() : _role(treeEnum::R_ROBOT_STOP) { }

role::role(const treeEnum& r)
{
    setRole(r);
}

role::~role() { }

treeEnum role::getRole() const
{
    return _role;
}

void role::setRole (const treeEnum& r)
{
    if ((r < treeEnum::ATTACKER_MAIN) || (r > treeEnum::R_ROBOT_STOP))
    {
        std::ostringstream msg;
        msg << "Error: attempt to set an invalid role";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }
    else
    {
        _role = r;
    }
}

boost::optional<treeEnum> role::getAssistantRole() const
{
    switch (_role)
    {
    case treeEnum::ATTACKER_MAIN:
        return treeEnum::ATTACKER_ASSIST;

    case treeEnum::ATTACKER_ASSIST:
        return treeEnum::ATTACKER_MAIN;

    case treeEnum::DEFENDER_MAIN:
        return treeEnum::DEFENDER_ASSIST;

    case treeEnum::DEFENDER_ASSIST:
        return treeEnum::DEFENDER_MAIN;

    default:
        return boost::none;
    }
}

bool role::isSafeOnMultipleRobots() const
{
    // Currently, only R_ROBOT_STOP is safe on multiple robots
    return (_role == treeEnum::R_ROBOT_STOP);
}

std::string role::str() const
{
    try
    {
        std::map<std::string, treeEnum>::const_iterator it;
        for(it = treeEnumMapping.begin(); it != treeEnumMapping.end(); ++it)
        {
            if(it->second == _role)
            {
                return it->first;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("Failed to parse treeEnum '" + std::to_string((int)_role) + "' to std::string. Linked to: " + e.what()));
    }
    return "INVALID";
}
