// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cGameStateTypes.hpp
 *
 *  Created on: Sep 8, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CGAMESTATETYPES_HPP_
#define CGAMESTATETYPES_HPP_

#include <string>
#include <map>
#include <boost/assign/list_of.hpp>

#include "int/types/cDecisionTreeTypes.hpp"

static std::map<std::string, treeEnum> gameStateEnumMapping = boost::assign::map_list_of
    ("INVALID", treeEnum::INVALID)
    ("NEUTRAL_STOPPED", treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL)
    ("NEUTRAL_PLAYING", treeEnum::IN_MATCH_NEUTRAL_PLAYING_NEUTRAL)
    ("OWN_KICKOFF_PREPARE", treeEnum::IN_MATCH_OWN_KICKOFF_PREPARE_NEUTRAL)
    ("OPP_KICKOFF_PREPARE", treeEnum::IN_MATCH_OPP_KICKOFF_PREPARE_NEUTRAL)
    ("OWN_KICKOFF_EXECUTE", treeEnum::IN_MATCH_OWN_KICKOFF_EXECUTE_NEUTRAL)
    ("OPP_KICKOFF_EXECUTE", treeEnum::IN_MATCH_OPP_KICKOFF_EXECUTE_NEUTRAL)
    ("OWN_FREEKICK_PREPARE", treeEnum::IN_MATCH_OWN_FREEKICK_PREPARE_NEUTRAL)
    ("OPP_FREEKICK_PREPARE", treeEnum::IN_MATCH_OPP_FREEKICK_PREPARE_NEUTRAL)
    ("OWN_FREEKICK_EXECUTE", treeEnum::IN_MATCH_OWN_FREEKICK_EXECUTE_NEUTRAL)
    ("OPP_FREEKICK_EXECUTE", treeEnum::IN_MATCH_OPP_FREEKICK_EXECUTE_NEUTRAL)
    ("OWN_GOALKICK_PREPARE", treeEnum::IN_MATCH_OWN_GOALKICK_PREPARE_NEUTRAL)
    ("OPP_GOALKICK_PREPARE", treeEnum::IN_MATCH_OPP_GOALKICK_PREPARE_NEUTRAL)
    ("OWN_GOALKICK_EXECUTE", treeEnum::IN_MATCH_OWN_GOALKICK_EXECUTE_NEUTRAL)
    ("OPP_GOALKICK_EXECUTE", treeEnum::IN_MATCH_OPP_GOALKICK_EXECUTE_NEUTRAL)
    ("OWN_THROWIN_PREPARE", treeEnum::IN_MATCH_OWN_THROWIN_PREPARE_NEUTRAL)
    ("OPP_THROWIN_PREPARE", treeEnum::IN_MATCH_OPP_THROWIN_PREPARE_NEUTRAL)
    ("OWN_THROWIN_EXECUTE", treeEnum::IN_MATCH_OWN_THROWIN_EXECUTE_NEUTRAL)
    ("OPP_THROWIN_EXECUTE", treeEnum::IN_MATCH_OPP_THROWIN_EXECUTE_NEUTRAL)
    ("OWN_CORNER_PREPARE", treeEnum::IN_MATCH_OWN_CORNER_PREPARE_NEUTRAL)
    ("OPP_CORNER_PREPARE", treeEnum::IN_MATCH_OPP_CORNER_PREPARE_NEUTRAL)
    ("OWN_CORNER_EXECUTE", treeEnum::IN_MATCH_OWN_CORNER_EXECUTE_NEUTRAL)
    ("OPP_CORNER_EXECUTE", treeEnum::IN_MATCH_OPP_CORNER_EXECUTE_NEUTRAL)
    ("OWN_PENALTY_PREPARE", treeEnum::IN_MATCH_OWN_PENALTY_PREPARE_NEUTRAL)
    ("OPP_PENALTY_PREPARE", treeEnum::IN_MATCH_OPP_PENALTY_PREPARE_NEUTRAL)
    ("OWN_PENALTY_EXECUTE", treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL)
    ("OPP_PENALTY_EXECUTE", treeEnum::IN_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL)
    ("DROPPED_BALL_PREPARE", treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL)
    ("DROPPED_BALL_EXECUTE", treeEnum::IN_MATCH_DROPPED_BALL_EXECUTE_NEUTRAL)
    ("OUT_OF_MATCH_NEUTRAL_STOPPED", treeEnum::OUT_OF_MATCH_NEUTRAL_STOPPED_NEUTRAL)
    ("OWN_PENALTY_SHOOTOUT_PREPARE", treeEnum::OUT_OF_MATCH_OWN_PENALTY_PREPARE_NEUTRAL)
    ("OPP_PENALTY_SHOOTOUT_PREPARE", treeEnum::OUT_OF_MATCH_OPP_PENALTY_PREPARE_NEUTRAL)
    ("OWN_PENALTY_SHOOTOUT_EXECUTE", treeEnum::OUT_OF_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL)
    ("OPP_PENALTY_SHOOTOUT_EXECUTE", treeEnum::OUT_OF_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL)
    ("PARKING", treeEnum::PARKING)
    ;

#endif /* CGAMESTATETYPES_HPP_ */
