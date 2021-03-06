// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldStateFunctionTypes.hpp
 *
 *  Created on: Jan 24, 2016
 *      Author: Tim Kouters
 */

#ifndef CWORLDSTATEFUNCTIONTYPES_HPP_
#define CWORLDSTATEFUNCTIONTYPES_HPP_

#include <string>
#include <map>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include "int/cWorldStateFunctions.hpp"

enum class worldStateFunctionEnum
{
	INVALID = 0,

	IS_LOWEST_ACTIVE_ROBOT,
	IS_HIGHEST_ACTIVE_ROBOT,
	IS_SECOND_HIGHEST_ACTIVE_ROBOT,
	IS_THIRD_HIGHEST_ACTIVE_ROBOT,
	IS_ONLY_ACTIVE_ROBOT,

	IS_IN_MATCH,

	IS_SETPIECE,
	IS_OWN_SETPIECE,
	IS_PREPARE_SETPIECE,
	IS_KICKOFF_SETPIECE,
	IS_DROPPEDBALL_SETPIECE,
	IS_PENALTY_SETPIECE,
	IS_GOALKICK_SETPIECE,
	IS_FREEKICK_SETPIECE,
	IS_CORNER_SETPIECE,
	IS_THROWIN_SETPIECE,
	IS_PARKING_SETPIECE,

	WITHIN_1M_OF_BALL,
	DOES_TEAM_HAVE_BALL,
	DOES_OWN_ROBOT_HAVE_BALL,
	IS_OWN_ROBOT_AT_OPPONENT_SIDE,
	IS_BALL_AT_SIDE,
	IS_OWN_ROBOT_CLOSEST_TO_POI,
	IS_OWN_ROBOT_SECOND_CLOSEST_TO_POI,
	IS_OWN_ROBOT_FURTHEST_FROM_POI,
	IS_OWN_ROBOT_SETPIECE_DEFENDER_ASSIST,
	IS_OWN_ROBOT_NEAREST_TO_BALL,
	IS_OWN_ROBOT_NEAREST_TO_LAST_KNOWN_BALL_LOCATION,
	IS_BALL_LOCATION_KNOWN,
	IS_BALL_IN_OWN_PENALTY_AREA,
	IS_BALL_IN_OPPONENT_PENALTY_AREA,
	IS_LAST_KNOWN_BALL_LOCATION_ON_THE_HALF_OF_MY_ROLE,

	IS_ASSISTENT_PRESENT,
	IS_POTENTIAL_OPP_ATTACKER_PRESENT,

	IS_SHORT_TURN_TO_GOAL_BLOCKED_BY_OPPONENT,
	IS_LONG_TURN_TO_GOAL_BLOCKED_BY_OPPONENT,

	IS_VALID_NUMBER_OF_PASSES_GIVEN,

	IS_OPPONENT_GOALKEEPER_IN_LEFT_CORNER,
	IS_OPPONENT_GOALKEEPER_IN_RIGHT_CORNER,

	IS_BALL_APPROACHING_ROBOT,
	IS_PASS_APPROACHING_ROBOT,
	IS_OPPONENT_HALF_REACHABLE,

	IS_IN_SCORING_POSITION,
	IS_SHOT_ON_GOAL_BLOCKED,
	IS_LOB_SHOT_ON_GOAL_BLOCKED,
	IS_PASS_TO_CLOSEST_TEAMMEMBER_BLOCKED,
	IS_PASS_TO_CLOSEST_ATTACKER_BLOCKED,
	IS_PASS_TO_FURTHEST_ATTACKER_BLOCKED,
	IS_PASS_TO_FURTHEST_DEFENDER_BLOCKED,
	IS_TIP_IN_BLOCKED,
	IS_PATH_TO_BALL_BLOCKED,

	DOES_ASSISTANT_HAVE_BALL,
	ALL_ROBOTS_ACTIVE,

	DEFENDING_STRATEGY_ON,

	MULTIPLE_OPPONENTS_ON_OWN_HALF,
	IS_AN_ATTACKER_ON_OPP_HALF,

	SIZE_OF_ENUM
};

static std::map<std::string, worldStateFunctionEnum> worldStateFunctionMappingStrToEnum = boost::assign::map_list_of
		("INVALID", worldStateFunctionEnum::INVALID)
		("IS_LOWEST_ACTIVE_ROBOT", worldStateFunctionEnum::IS_LOWEST_ACTIVE_ROBOT)
		("IS_HIGHEST_ACTIVE_ROBOT", worldStateFunctionEnum::IS_HIGHEST_ACTIVE_ROBOT)
		("IS_SECOND_HIGHEST_ACTIVE_ROBOT", worldStateFunctionEnum::IS_SECOND_HIGHEST_ACTIVE_ROBOT)
		("IS_THIRD_HIGHEST_ACTIVE_ROBOT", worldStateFunctionEnum::IS_THIRD_HIGHEST_ACTIVE_ROBOT)
		("IS_ONLY_ACTIVE_ROBOT", worldStateFunctionEnum::IS_ONLY_ACTIVE_ROBOT)
		("IS_IN_MATCH", worldStateFunctionEnum::IS_IN_MATCH)
		("IS_SETPIECE", worldStateFunctionEnum::IS_SETPIECE)
		("IS_OWN_SETPIECE", worldStateFunctionEnum::IS_OWN_SETPIECE)
		("IS_PREPARE_SETPIECE", worldStateFunctionEnum::IS_PREPARE_SETPIECE)
		("IS_KICKOFF_SETPIECE", worldStateFunctionEnum::IS_KICKOFF_SETPIECE)
		("IS_DROPPEDBALL_SETPIECE", worldStateFunctionEnum::IS_DROPPEDBALL_SETPIECE)
		("IS_PENALTY_SETPIECE", worldStateFunctionEnum::IS_PENALTY_SETPIECE)
		("IS_GOALKICK_SETPIECE", worldStateFunctionEnum::IS_GOALKICK_SETPIECE)
		("IS_FREEKICK_SETPIECE", worldStateFunctionEnum::IS_FREEKICK_SETPIECE)
		("IS_CORNER_SETPIECE", worldStateFunctionEnum::IS_CORNER_SETPIECE)
		("IS_THROWIN_SETPIECE", worldStateFunctionEnum::IS_THROWIN_SETPIECE)
		("IS_PARKING_SETPIECE", worldStateFunctionEnum::IS_PARKING_SETPIECE)
		("WITHIN_1M_OF_BALL", worldStateFunctionEnum::WITHIN_1M_OF_BALL)
		("DOES_TEAM_HAVE_BALL", worldStateFunctionEnum::DOES_TEAM_HAVE_BALL)
		("DOES_OWN_ROBOT_HAVE_BALL", worldStateFunctionEnum::DOES_OWN_ROBOT_HAVE_BALL)
		("IS_OWN_ROBOT_AT_OPPONENT_SIDE", worldStateFunctionEnum::IS_OWN_ROBOT_AT_OPPONENT_SIDE)
		("IS_BALL_AT_SIDE", worldStateFunctionEnum::IS_BALL_AT_SIDE)
		("IS_OWN_ROBOT_CLOSEST_TO_POI", worldStateFunctionEnum::IS_OWN_ROBOT_CLOSEST_TO_POI)
		("IS_OWN_ROBOT_SECOND_CLOSEST_TO_POI", worldStateFunctionEnum::IS_OWN_ROBOT_SECOND_CLOSEST_TO_POI)
		("IS_OWN_ROBOT_FURTHEST_FROM_POI", worldStateFunctionEnum::IS_OWN_ROBOT_FURTHEST_FROM_POI)
		("IS_OWN_ROBOT_SETPIECE_DEFENDER_ASSIST", worldStateFunctionEnum::IS_OWN_ROBOT_SETPIECE_DEFENDER_ASSIST)
		("IS_OWN_ROBOT_NEAREST_TO_BALL", worldStateFunctionEnum::IS_OWN_ROBOT_NEAREST_TO_BALL)
		("IS_OWN_ROBOT_NEAREST_TO_LAST_KNOWN_BALL_LOCATION", worldStateFunctionEnum::IS_OWN_ROBOT_NEAREST_TO_LAST_KNOWN_BALL_LOCATION)
		("IS_BALL_LOCATION_KNOWN", worldStateFunctionEnum::IS_BALL_LOCATION_KNOWN)
		("IS_BALL_IN_OWN_PENALTY_AREA", worldStateFunctionEnum::IS_BALL_IN_OWN_PENALTY_AREA)
		("IS_BALL_IN_OPPONENT_PENALTY_AREA", worldStateFunctionEnum::IS_BALL_IN_OPPONENT_PENALTY_AREA)
		("IS_LAST_KNOWN_BALL_LOCATION_ON_THE_HALF_OF_MY_ROLE", worldStateFunctionEnum::IS_LAST_KNOWN_BALL_LOCATION_ON_THE_HALF_OF_MY_ROLE)
		("IS_ASSISTENT_PRESENT", worldStateFunctionEnum::IS_ASSISTENT_PRESENT)
		("IS_POTENTIAL_OPP_ATTACKER_PRESENT", worldStateFunctionEnum::IS_POTENTIAL_OPP_ATTACKER_PRESENT)
		("IS_SHORT_TURN_TO_GOAL_BLOCKED_BY_OPPONENT", worldStateFunctionEnum::IS_SHORT_TURN_TO_GOAL_BLOCKED_BY_OPPONENT)
		("IS_LONG_TURN_TO_GOAL_BLOCKED_BY_OPPONENT", worldStateFunctionEnum::IS_LONG_TURN_TO_GOAL_BLOCKED_BY_OPPONENT)
		("IS_VALID_NUMBER_OF_PASSES_GIVEN", worldStateFunctionEnum::IS_VALID_NUMBER_OF_PASSES_GIVEN)
		("IS_OPPONENT_GOALKEEPER_IN_LEFT_CORNER", worldStateFunctionEnum::IS_OPPONENT_GOALKEEPER_IN_LEFT_CORNER)
		("IS_OPPONENT_GOALKEEPER_IN_RIGHT_CORNER", worldStateFunctionEnum::IS_OPPONENT_GOALKEEPER_IN_RIGHT_CORNER)
		("IS_BALL_APPROACHING_ROBOT", worldStateFunctionEnum::IS_BALL_APPROACHING_ROBOT)
		("IS_PASS_APPROACHING_ROBOT", worldStateFunctionEnum::IS_PASS_APPROACHING_ROBOT)
		("IS_OPPONENT_HALF_REACHABLE", worldStateFunctionEnum::IS_OPPONENT_HALF_REACHABLE)
		("IS_IN_SCORING_POSITION", worldStateFunctionEnum::IS_IN_SCORING_POSITION)
		("IS_SHOT_ON_GOAL_BLOCKED", worldStateFunctionEnum::IS_SHOT_ON_GOAL_BLOCKED)
		("IS_LOB_SHOT_ON_GOAL_BLOCKED", worldStateFunctionEnum::IS_LOB_SHOT_ON_GOAL_BLOCKED)
		("IS_PASS_TO_CLOSEST_TEAMMEMBER_BLOCKED", worldStateFunctionEnum::IS_PASS_TO_CLOSEST_TEAMMEMBER_BLOCKED)
		("IS_PASS_TO_CLOSEST_ATTACKER_BLOCKED", worldStateFunctionEnum::IS_PASS_TO_CLOSEST_ATTACKER_BLOCKED)
		("IS_PASS_TO_FURTHEST_ATTACKER_BLOCKED", worldStateFunctionEnum::IS_PASS_TO_FURTHEST_ATTACKER_BLOCKED)
		("IS_PASS_TO_FURTHEST_DEFENDER_BLOCKED", worldStateFunctionEnum::IS_PASS_TO_FURTHEST_DEFENDER_BLOCKED)
		("IS_TIP_IN_BLOCKED", worldStateFunctionEnum::IS_TIP_IN_BLOCKED)
		("IS_PATH_TO_BALL_BLOCKED", worldStateFunctionEnum::IS_PATH_TO_BALL_BLOCKED)
		("DOES_ASSISTANT_HAVE_BALL", worldStateFunctionEnum::DOES_ASSISTANT_HAVE_BALL)
		("ALL_ROBOTS_ACTIVE", worldStateFunctionEnum::ALL_ROBOTS_ACTIVE)
		("DEFENDING_STRATEGY_ON", worldStateFunctionEnum::DEFENDING_STRATEGY_ON)
		("MULTIPLE_OPPONENTS_ON_OWN_HALF", worldStateFunctionEnum::MULTIPLE_OPPONENTS_ON_OWN_HALF)
		("IS_AN_ATTACKER_ON_OPP_HALF", worldStateFunctionEnum::IS_AN_ATTACKER_ON_OPP_HALF)
		;

typedef boost::function<bool(const std::map<std::string, std::string>&)> worldstateFunctionType;

#endif /* CWORLDSTATEFUNCTIONTYPES_HPP_ */
