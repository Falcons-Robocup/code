// Copyright 2016-2020 martijn van veen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldStateFunctionsTypesFunctions.hpp
 * Contains simple conversion functions that are only used in some callers, to avoid w_unused function from many includes
 *
 *  Created on: July 7, 2016
 *      Author: Martijn van Veen
 */

#ifndef CWORLDSTATEFUNCTIONTYPESFUNCTIONS_HPP_
#define CWORLDSTATEFUNCTIONTYPESFUNCTIONS_HPP_

#include "int/cWorldStateFunctions.hpp"

typedef boost::function<bool(const std::map<std::string, std::string>&)> worldstateFunctionType;

std::map<worldStateFunctionEnum, worldstateFunctionType> worldStateFunctionMappingEnumToFunc = boost::assign::map_list_of
            (worldStateFunctionEnum::IS_LOWEST_ACTIVE_ROBOT, boost::bind(&isLowestActiveRobotID, _1))
            (worldStateFunctionEnum::IS_HIGHEST_ACTIVE_ROBOT, boost::bind(&isHighestActiveRobotID, _1))
            (worldStateFunctionEnum::IS_SECOND_HIGHEST_ACTIVE_ROBOT, boost::bind(&isSecondHighestActiveRobotID, _1))
            (worldStateFunctionEnum::IS_THIRD_HIGHEST_ACTIVE_ROBOT, boost::bind(&isThirdHighestActiveRobotID, _1))
			(worldStateFunctionEnum::IS_ONLY_ACTIVE_ROBOT, boost::bind(&isOnlyActiveRobotID, _1))
            (worldStateFunctionEnum::IS_IN_MATCH, boost::bind(&isInMatch, _1))
            (worldStateFunctionEnum::IS_SETPIECE, boost::bind(&isSetPiece, _1))
            (worldStateFunctionEnum::IS_OWN_SETPIECE, boost::bind(&isOwnSetPiece, _1))
            (worldStateFunctionEnum::IS_PREPARE_SETPIECE, boost::bind(&isPrepareSetPiece, _1))
            (worldStateFunctionEnum::IS_KICKOFF_SETPIECE, boost::bind(&isKickoffSetPiece, _1))
            (worldStateFunctionEnum::IS_DROPPEDBALL_SETPIECE, boost::bind(&isDroppedBallSetPiece, _1))
            (worldStateFunctionEnum::IS_PENALTY_SETPIECE, boost::bind(&isPenaltySetPiece, _1))
            (worldStateFunctionEnum::IS_GOALKICK_SETPIECE, boost::bind(&isGoalkickSetPiece, _1))
            (worldStateFunctionEnum::IS_FREEKICK_SETPIECE, boost::bind(&isFreekickSetPiece, _1))
            (worldStateFunctionEnum::IS_CORNER_SETPIECE, boost::bind(&isCornerSetPiece, _1))
            (worldStateFunctionEnum::IS_THROWIN_SETPIECE, boost::bind(&isThrowinSetPiece, _1))
            (worldStateFunctionEnum::IS_PARKING_SETPIECE, boost::bind(&isParkingSetPiece, _1))
            (worldStateFunctionEnum::WITHIN_1M_OF_BALL, boost::bind(&within1mOfBall, _1))
            (worldStateFunctionEnum::DOES_TEAM_HAVE_BALL, boost::bind(&doesTeamHaveBall, _1))
            (worldStateFunctionEnum::DOES_OWN_ROBOT_HAVE_BALL, boost::bind(&doesOwnRobotHaveBall, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_AT_OPPONENT_SIDE, boost::bind(&isOwnRobotAtOpponentSide, _1))
            (worldStateFunctionEnum::IS_BALL_AT_SIDE, boost::bind(&isBallAtSide, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_CLOSEST_TO_POI, boost::bind(&isOwnRobotClosestToPOI, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_SECOND_CLOSEST_TO_POI, boost::bind(&isOwnRobotSecondClosestToPOI, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_FURTHEST_FROM_POI, boost::bind(&isOwnRobotFurthestFromPOI, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_SETPIECE_DEFENDER_ASSIST, boost::bind(&isOwnRobotSetpieceDefenderAssist, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_NEAREST_TO_BALL, boost::bind(&isOwnRobotNearestToBall, _1))
            (worldStateFunctionEnum::IS_OWN_ROBOT_NEAREST_TO_LAST_KNOWN_BALL_LOCATION, boost::bind(&isOwnRobotNearestToLastKnownBallLocation, _1))
            (worldStateFunctionEnum::IS_BALL_LOCATION_KNOWN, boost::bind(&isBallLocationKnown, _1))
            (worldStateFunctionEnum::IS_BALL_IN_OWN_PENALTY_AREA, boost::bind(&isBallInOwnPenaltyArea, _1))
            (worldStateFunctionEnum::IS_BALL_IN_OPPONENT_PENALTY_AREA, boost::bind(&isBallInOpponentPenaltyArea, _1))
            (worldStateFunctionEnum::IS_ASSISTENT_PRESENT, boost::bind(&isAssistentPresent, _1))
            (worldStateFunctionEnum::IS_POTENTIAL_OPP_ATTACKER_PRESENT, boost::bind(&isPotentialOppAttackerPresent, _1))
            (worldStateFunctionEnum::IS_SHORT_TURN_TO_GOAL_BLOCKED_BY_OPPONENT, boost::bind(&isShortTurnToGoalBlockedByOpponent, _1))
            (worldStateFunctionEnum::IS_LONG_TURN_TO_GOAL_BLOCKED_BY_OPPONENT, boost::bind(&isLongTurnToGoalBlockedByOpponent, _1))
            (worldStateFunctionEnum::IS_VALID_NUMBER_OF_PASSES_GIVEN, boost::bind(&isValidNumberOfPassesGiven, _1))
            (worldStateFunctionEnum::IS_OPPONENT_GOALKEEPER_IN_LEFT_CORNER, boost::bind(&isOpponentGoalKeeperInLeftCorner, _1))
            (worldStateFunctionEnum::IS_OPPONENT_GOALKEEPER_IN_RIGHT_CORNER, boost::bind(&isOpponentGoalKeeperInRightCorner, _1))
            (worldStateFunctionEnum::IS_BALL_APPROACHING_ROBOT, boost::bind(&isBallApproachingRobot, _1))
            (worldStateFunctionEnum::IS_PASS_APPROACHING_ROBOT, boost::bind(&isPassApproachingRobot, _1))
            (worldStateFunctionEnum::IS_OPPONENT_HALF_REACHABLE, boost::bind(&isOpponentHalfReachable, _1))
            (worldStateFunctionEnum::IS_IN_SCORING_POSITION, boost::bind(&isInScoringPosition, _1))
            (worldStateFunctionEnum::IS_SHOT_ON_GOAL_BLOCKED, boost::bind(&isShotOnGoalBlocked, _1))
            (worldStateFunctionEnum::IS_LOB_SHOT_ON_GOAL_BLOCKED, boost::bind(&isLobShotOnGoalBlocked, _1))
            (worldStateFunctionEnum::IS_PASS_TO_CLOSEST_TEAMMEMBER_BLOCKED, boost::bind(&isPassToClosestTeammemberBlocked, _1))
            (worldStateFunctionEnum::IS_PASS_TO_CLOSEST_ATTACKER_BLOCKED, boost::bind(&isPassToClosestAttackerBlocked, _1))
            (worldStateFunctionEnum::IS_PASS_TO_FURTHEST_ATTACKER_BLOCKED, boost::bind(&isPassToFurthestAttackerBlocked, _1))
            (worldStateFunctionEnum::IS_PASS_TO_FURTHEST_DEFENDER_BLOCKED, boost::bind(&isPassToFurthestDefenderBlocked, _1))
            (worldStateFunctionEnum::IS_TIP_IN_BLOCKED, boost::bind(&isTipInBlocked, _1))
            (worldStateFunctionEnum::IS_PATH_TO_BALL_BLOCKED, boost::bind(&isPathToBallBlocked, _1))
            (worldStateFunctionEnum::DOES_ASSISTANT_HAVE_BALL, boost::bind(&doesAssistantHaveBall, _1))
            (worldStateFunctionEnum::ALL_ROBOTS_ACTIVE, boost::bind(&allRobotsActive, _1))
            (worldStateFunctionEnum::DEFENDING_STRATEGY_ON, boost::bind(&defendingStrategyOn, _1))
            (worldStateFunctionEnum::MULTIPLE_OPPONENTS_ON_OWN_HALF, boost::bind(&multipleOpponentsOnOwnHalf, _1))
            (worldStateFunctionEnum::IS_AN_ATTACKER_ON_OPP_HALF, boost::bind(&isAnAttackerOnOppHalf, _1))
            ;


#endif /* CWORLDSTATEFUNCTIONTYPESFUNCTIONS_HPP_ */
