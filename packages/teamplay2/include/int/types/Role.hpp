// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Role.hpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef ROLE_HPP_
#define ROLE_HPP_

#include <string>
#include "boost/optional.hpp"

namespace teamplay
{

enum class RoleEnum {
    INVALID,
    STOP,
    GOALKEEPER,
    ATTACKER_MAIN,
    ATTACKER_ASSIST,
    DEFENDER_MAIN,
    DEFENDER_ASSIST
};

inline char const *enum2str(RoleEnum const& role) // TODO: move RoleEnum to sharedTypes and remove this handcrafted function
{
    char const *result = "INVALID";
    switch (role)
    {
        case RoleEnum::STOP:
            result = "STOP";
            break;
        case RoleEnum::GOALKEEPER:
            result = "GOALKEEPER";
            break;
        case RoleEnum::ATTACKER_MAIN:
            result = "ATTACKER_MAIN";
            break;
        case RoleEnum::ATTACKER_ASSIST:
            result = "ATTACKER_ASSIST";
            break;
        case RoleEnum::DEFENDER_MAIN:
            result = "DEFENDER_MAIN";
            break;
        case RoleEnum::DEFENDER_ASSIST:
            result = "DEFENDER_ASSIST";
            break;

        default:
            result = "INVALID";
    }
    return result;
}

inline RoleEnum str2enum(const std::string& roleStr) // TODO: move RoleEnum to sharedTypes and remove this handcrafted function
{
    //enum class RoleEnum <-- MTP 
    //{
    //    UNDEFINED = 0,
    //    GOALKEEPER = 1,
    //    ATTACKER_MAIN = 2,
    //    ATTACKER_ASSIST = 3,
    //    ATTACKER_GENERIC = 4,
    //    DEFENDER_MAIN = 5,
    //    DEFENDER_GENERIC = 6,
    //    DISABLED_OUT = 7,
    //    DISABLED_IN = 8
    //}; // end of enum class RoleEnum
    teamplay::RoleEnum result = teamplay::RoleEnum::INVALID;
    if (roleStr == "ROBOT_STOP") result = teamplay::RoleEnum::STOP;
    if (roleStr == "GOALKEEPER") result = teamplay::RoleEnum::GOALKEEPER;
    if (roleStr == "ATTACKER_MAIN") result = teamplay::RoleEnum::ATTACKER_MAIN;
    if (roleStr == "ATTACKER_ASSIST") result = teamplay::RoleEnum::ATTACKER_ASSIST;
    if (roleStr == "ATTACKER_GENERIC") result = teamplay::RoleEnum::ATTACKER_ASSIST;
    if (roleStr == "DEFENDER_MAIN") result = teamplay::RoleEnum::DEFENDER_MAIN;
    if (roleStr == "DEFENDER_ASSIST") result = teamplay::RoleEnum::DEFENDER_ASSIST;
    if (roleStr == "DEFENDER_GENERIC") result = teamplay::RoleEnum::DEFENDER_ASSIST;
    if (roleStr == "DISABLED_OUT") result = teamplay::RoleEnum::STOP;
    if (roleStr == "DISABLED_IN") result = teamplay::RoleEnum::STOP;
    return result;
}


class Role {
public:
    Role();
    Role(const RoleEnum&);
    virtual ~Role();

    virtual RoleEnum getRole() const;
    virtual boost::optional<RoleEnum> getAssistantRole() const;

    virtual void setRole (const RoleEnum&);

    virtual std::string str() const;

private:
    RoleEnum _role;
};

} /* namespace teamplay */

#endif /* ROLE_HPP_ */
