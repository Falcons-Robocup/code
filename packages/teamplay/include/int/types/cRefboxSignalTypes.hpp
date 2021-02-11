// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRefboxSignalTypes.hpp
 *
 *  Created on: Sep 8, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CREFBOXSIGNALTYPES_HPP_
#define CREFBOXSIGNALTYPES_HPP_

#include <string>
#include <map>
#include <boost/assign/list_of.hpp>

enum class refboxSignalEnum
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

    SIZE_OF_ENUM
};

static std::map<std::string, refboxSignalEnum> refboxSignalMapping = boost::assign::map_list_of
    ("INVALID", refboxSignalEnum::INVALID)
    ("STOP", refboxSignalEnum::STOP)
    ("HALT", refboxSignalEnum::HALT)
    ("CANCEL", refboxSignalEnum::CANCEL)
    ("READY", refboxSignalEnum::READY)
    ("START", refboxSignalEnum::START)
    ("RESTART", refboxSignalEnum::RESTART)
    ("FIRST_HALF", refboxSignalEnum::FIRST_HALF)
    ("HALF_TIME", refboxSignalEnum::HALF_TIME)
    ("SECOND_HALF", refboxSignalEnum::SECOND_HALF)
    ("END_GAME", refboxSignalEnum::END_GAME)
    ("KICKOFF_OWN", refboxSignalEnum::KICKOFF_OWN)
    ("KICKOFF_OPP", refboxSignalEnum::KICKOFF_OPP)
    ("FREEKICK_OWN", refboxSignalEnum::FREE_KICK_OWN)
    ("FREEKICK_OPP", refboxSignalEnum::FREE_KICK_OPP)
    ("GOALKICK_OWN", refboxSignalEnum::GOAL_KICK_OWN)
    ("GOALKICK_OPP", refboxSignalEnum::GOAL_KICK_OPP)
    ("THROWIN_OWN", refboxSignalEnum::THROWIN_KICK_OWN)
    ("THROWIN_OPP", refboxSignalEnum::THROWIN_KICK_OPP)
    ("CORNER_OWN", refboxSignalEnum::CORNER_KICK_OWN)
    ("CORNER_OPP", refboxSignalEnum::CORNER_KICK_OPP)
    ("PENALTY_OWN", refboxSignalEnum::PENALTY_KICK_OWN)
    ("PENALTY_OPP", refboxSignalEnum::PENALTY_KICK_OPP)
    ("GOAL_OWN", refboxSignalEnum::GOAL_OWN)
    ("GOAL_OPP", refboxSignalEnum::GOAL_OPP)
    ("SUBGOAL_OWN", refboxSignalEnum::SUBGOAL_OWN)
    ("SUBGOAL_OPP", refboxSignalEnum::SUBGOAL_OPP)
    ("DUMMY", refboxSignalEnum::DUMMY)
    ("DROPPED_BALL", refboxSignalEnum::DROPPED_BALL)
    ("PARK", refboxSignalEnum::PARK)
    ("SUBSTITUTION_OWN", refboxSignalEnum::SUBSTITUTION_OWN)
    // note SUBSTITUTION_OPP is filtered at refboxRelay to prevent it interfering with SUBSTITUTION_OWN
    // because only the last command would end up here in teamplay...
    // if this is ever needed in teamplay (?), then refboxRelay and the communication to teamplay
    // must be made robust for rapid bursts of different refbox commands at once
    ;

#endif /* CREFBOXSIGNALTYPES_HPP_ */


