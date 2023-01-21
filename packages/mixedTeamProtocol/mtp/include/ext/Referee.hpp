// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_REFEREE_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_REFEREE_HPP_

// standard/system headers
#include <vector>
#include <string>
#include <map>

namespace mtp
{

// explicit numbering for compatibility reasons
enum class CommandEnum
{
    NONE = 0,
    START = 1,
    STOP = 2,
    DROP_BALL = 3,
    HALF_TIME = 4,
    END_GAME = 5,
    GAME_OVER = 6,
    PARK = 7,
    FIRST_HALF = 8,
    SECOND_HALF = 9,
    FIRST_HALF_OVERTIME = 10,
    SECOND_HALF_OVERTIME = 11,
    RESET = 12,
    WELCOME = 13,
    KICKOFF = 14,
    FREEKICK = 15,
    GOALKICK = 16,
    THROWIN = 17,
    CORNER = 18,
    PENALTY = 19,
    GOAL = 20,
    SUBGOAL = 21,
    REPAIR = 22,
    YELLOW_CARD = 23,
    DOUBLE_YELLOW = 24,
    RED_CARD = 25,
    SUBSTITUTION = 26,
    IS_ALIVE = 27
}; // end of enum class CommandEnum

enum class TargetEnum
{
    ALL = 0,
    US = 1,
    THEM = 2
};

struct RefereeCommand
{
    using Arguments = std::map<std::string,std::string>;

    CommandEnum command = CommandEnum::NONE;
    TargetEnum target = TargetEnum::ALL;
    Arguments arguments;
}; // end of class Command

// conversion functions
std::string commandEnumToString(CommandEnum &c);
CommandEnum commandStringToEnum(std::string s);
std::string targetEnumToString(TargetEnum &c);
TargetEnum targetStringToEnum(std::string s);

} // end of namespace mtp

#endif
