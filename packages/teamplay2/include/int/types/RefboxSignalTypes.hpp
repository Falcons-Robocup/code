// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRefboxSignalTypes.hpp
 *
 *  Created on: Sep 8, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef REFBOXSIGNALTYPES_HPP_
#define REFBOXSIGNALTYPES_HPP_

#include <iostream>
#include <string>
#include <map>
#include <boost/assign/list_of.hpp>

using RefboxSignalArguments = std::map<std::string, std::string>;

enum class RefboxSignalEnum
{
    INVALID = 0,
    STOP,
    HALT,
    CANCEL,
    READY,
    START,
    RESTART,
    FIRST_HALF,
    HALF_TIME,
    SECOND_HALF,
    END_GAME,
    KICKOFF_OWN,
    KICKOFF_OPP,
    FREE_KICK_OWN,
    FREE_KICK_OPP,
    GOAL_KICK_OWN,
    GOAL_KICK_OPP,
    THROWIN_KICK_OWN,
    THROWIN_KICK_OPP,
    CORNER_KICK_OWN,
    CORNER_KICK_OPP,
    PENALTY_KICK_OWN,
    PENALTY_KICK_OPP,
    GOAL_OWN,
    GOAL_OPP,
    SUBGOAL_OWN,
    SUBGOAL_OPP,
    DUMMY,
    DROPPED_BALL,
    PARK,
    SUBSTITUTION_OWN,
    IS_ALIVE_OWN,

    SIZE_OF_ENUM
};

static std::map<std::string, RefboxSignalEnum> refboxSignalMapping = boost::assign::map_list_of
    ("INVALID", RefboxSignalEnum::INVALID)
    ("STOP", RefboxSignalEnum::STOP)
    ("HALT", RefboxSignalEnum::HALT)
    ("CANCEL", RefboxSignalEnum::CANCEL)
    ("READY", RefboxSignalEnum::READY)
    ("START", RefboxSignalEnum::START)
    ("RESTART", RefboxSignalEnum::RESTART)
    ("FIRST_HALF", RefboxSignalEnum::FIRST_HALF)
    ("HALF_TIME", RefboxSignalEnum::HALF_TIME)
    ("SECOND_HALF", RefboxSignalEnum::SECOND_HALF)
    ("END_GAME", RefboxSignalEnum::END_GAME)
    ("KICKOFF_OWN", RefboxSignalEnum::KICKOFF_OWN)
    ("KICKOFF_OPP", RefboxSignalEnum::KICKOFF_OPP)
    ("FREEKICK_OWN", RefboxSignalEnum::FREE_KICK_OWN)
    ("FREEKICK_OPP", RefboxSignalEnum::FREE_KICK_OPP)
    ("GOALKICK_OWN", RefboxSignalEnum::GOAL_KICK_OWN)
    ("GOALKICK_OPP", RefboxSignalEnum::GOAL_KICK_OPP)
    ("THROWIN_OWN", RefboxSignalEnum::THROWIN_KICK_OWN)
    ("THROWIN_OPP", RefboxSignalEnum::THROWIN_KICK_OPP)
    ("CORNER_OWN", RefboxSignalEnum::CORNER_KICK_OWN)
    ("CORNER_OPP", RefboxSignalEnum::CORNER_KICK_OPP)
    ("PENALTY_OWN", RefboxSignalEnum::PENALTY_KICK_OWN)
    ("PENALTY_OPP", RefboxSignalEnum::PENALTY_KICK_OPP)
    ("GOAL_OWN", RefboxSignalEnum::GOAL_OWN)
    ("GOAL_OPP", RefboxSignalEnum::GOAL_OPP)
    ("SUBGOAL_OWN", RefboxSignalEnum::SUBGOAL_OWN)
    ("SUBGOAL_OPP", RefboxSignalEnum::SUBGOAL_OPP)
    ("DUMMY", RefboxSignalEnum::DUMMY)
    ("DROPPED_BALL", RefboxSignalEnum::DROPPED_BALL)
    ("PARK", RefboxSignalEnum::PARK)
    ("SUBSTITUTION_OWN", RefboxSignalEnum::SUBSTITUTION_OWN)
    // note SUBSTITUTION_OPP is filtered at refboxRelay to prevent it interfering with SUBSTITUTION_OWN
    // because only the last command would end up here in teamplay...
    // if this is ever needed in teamplay (?), then refboxRelay and the communication to teamplay
    // must be made robust for rapid bursts of different refbox commands at once
    ("IS_ALIVE_OWN", RefboxSignalEnum::IS_ALIVE_OWN)
    ;

#endif /* REFBOXSIGNALTYPES_HPP_ */
