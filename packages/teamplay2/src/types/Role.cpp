// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Role.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <string>

#include "int/types/Role.hpp"
#include "cDiagnostics.hpp"

using namespace teamplay;


Role::Role() : _role(RoleEnum::STOP) { }

Role::Role(const RoleEnum& r)
{
    setRole(r);
}

Role::~Role() { }

RoleEnum Role::getRole() const
{
    return _role;
}

void Role::setRole (const RoleEnum& r)
{
    _role = r;
}

boost::optional<RoleEnum> Role::getAssistantRole() const
{
    switch (_role)
    {
    case RoleEnum::ATTACKER_MAIN:
        return RoleEnum::ATTACKER_ASSIST;

    case RoleEnum::ATTACKER_ASSIST:
        return RoleEnum::ATTACKER_MAIN;

    case RoleEnum::DEFENDER_MAIN:
        return RoleEnum::DEFENDER_ASSIST;

    case RoleEnum::DEFENDER_ASSIST:
        return RoleEnum::DEFENDER_MAIN;

    default:
        return boost::none;
    }
}

std::string Role::str() const
{
    std::string result;
    switch(_role)
    {
        case RoleEnum::INVALID:
        {
            result = "Invalid Role";
            break;
        }
        case RoleEnum::STOP:
        {
            result = "Stop";
            break;
        }
        case RoleEnum::GOALKEEPER:
        {
            result = "Goalkeeper";
            break;
        }
        case RoleEnum::ATTACKER_MAIN:
        {
            result = "Attacker Main";
            break;
        }
        case RoleEnum::ATTACKER_ASSIST:
        {
            result = "Attacker Assist";
            break;
        }
        case RoleEnum::DEFENDER_MAIN:
        {
            result = "Defender Main";
            break;
        }
        case RoleEnum::DEFENDER_ASSIST:
        {
            result = "Defender Assist";
            break;
        }
        default:
        {
            result = "Unknown Role";
            break;
        }
    }
    return result;
}