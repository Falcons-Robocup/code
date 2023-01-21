// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "ext/Referee.hpp"

// standard/system headers
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <iostream>

using namespace mtp;


// enum/string conversions

typedef boost::bimap<CommandEnum, std::string> bm_command_type;
const bm_command_type commandEnumToStringBiMap =
  boost::assign::list_of<bm_command_type::relation>
    (CommandEnum::NONE, std::string("NONE"))
    (CommandEnum::START, std::string("START"))
    (CommandEnum::STOP, std::string("STOP"))
    (CommandEnum::DROP_BALL, std::string("DROP_BALL"))
    (CommandEnum::HALF_TIME, std::string("HALF_TIME"))
    (CommandEnum::END_GAME, std::string("END_GAME"))
    (CommandEnum::GAME_OVER, std::string("GAME_OVER"))
    (CommandEnum::PARK, std::string("PARK"))
    (CommandEnum::FIRST_HALF, std::string("FIRST_HALF"))
    (CommandEnum::SECOND_HALF, std::string("SECOND_HALF"))
    (CommandEnum::FIRST_HALF_OVERTIME, std::string("FIRST_HALF_OVERTIME"))
    (CommandEnum::SECOND_HALF_OVERTIME, std::string("SECOND_HALF_OVERTIME"))
    (CommandEnum::RESET, std::string("RESET"))
    (CommandEnum::WELCOME, std::string("WELCOME"))
    (CommandEnum::KICKOFF, std::string("KICKOFF"))
    (CommandEnum::FREEKICK, std::string("FREEKICK"))
    (CommandEnum::GOALKICK, std::string("GOALKICK"))
    (CommandEnum::THROWIN, std::string("THROWIN"))
    (CommandEnum::CORNER, std::string("CORNER"))
    (CommandEnum::PENALTY, std::string("PENALTY"))
    (CommandEnum::GOAL, std::string("GOAL"))
    (CommandEnum::SUBGOAL, std::string("SUBGOAL"))
    (CommandEnum::REPAIR, std::string("REPAIR"))
    (CommandEnum::YELLOW_CARD, std::string("YELLOW_CARD"))
    (CommandEnum::DOUBLE_YELLOW, std::string("DOUBLE_YELLOW"))
    (CommandEnum::RED_CARD, std::string("RED_CARD"))
    (CommandEnum::SUBSTITUTION, std::string("SUBSTITUTION"))
    (CommandEnum::IS_ALIVE, std::string("IS_ALIVE"));

typedef boost::bimap<TargetEnum, std::string> bm_target_type;
const bm_target_type targetEnumToStringBiMap =
  boost::assign::list_of<bm_target_type::relation>
    (TargetEnum::ALL, std::string("ALL"))
    (TargetEnum::US, std::string("US"))
    (TargetEnum::THEM, std::string("THEM"));

std::string mtp::commandEnumToString(CommandEnum &c)
{
    return commandEnumToStringBiMap.left.at(c);
}

CommandEnum mtp::commandStringToEnum(std::string s)
{
    if (s.compare("") == 0)
    {
        return CommandEnum::NONE;
    }
    bm_command_type::right_const_iterator it = commandEnumToStringBiMap.right.find(s);
    if (it == commandEnumToStringBiMap.right.end())
    {
        std::cerr << "commandStringToEnum| Not found: " << s << std::endl;
        return CommandEnum::NONE;
    }
    return commandEnumToStringBiMap.right.at(s);
}

std::string mtp::targetEnumToString(TargetEnum &t)
{
    return targetEnumToStringBiMap.left.at(t);
}

TargetEnum mtp::targetStringToEnum(std::string s)
{
    if (s.compare("") == 0)
    {
        return TargetEnum::ALL;
    }
    bm_target_type::right_const_iterator it = targetEnumToStringBiMap.right.find(s);
    if (it == targetEnumToStringBiMap.right.end())
    {
        std::cerr << "targetStringToEnum| Not found: " << s << std::endl;
        return TargetEnum::ALL;
    }
    return targetEnumToStringBiMap.right.at(s);
}
