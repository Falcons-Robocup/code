 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
            (worldStateFunctionEnum::DRIBBLE_STRATEGY_ON, boost::bind(&dribbleStrategyOn, _1))
            (worldStateFunctionEnum::MULTIPLE_OPPONENTS_ON_OWN_HALF, boost::bind(&multipleOpponentsOnOwnHalf, _1))
            (worldStateFunctionEnum::IS_AN_ATTACKER_ON_OPP_HALF, boost::bind(&isAnAttackerOnOppHalf, _1))
            (worldStateFunctionEnum::SHOT_THRESHOLD_REACHED, boost::bind(&shotThresholdReached, _1))
            (worldStateFunctionEnum::SHOT_THRESHOLD_REACHABLE, boost::bind(&shotThresholdReachable, _1))
            ;

/*
 // looked useful but is not used
static worldStateFunctionEnum worldStateFunctionStrToEnum(const std::string enumString)
{
    try
    {
        const worldStateFunctionEnum retVal = worldStateFunctionMappingStrToEnum.at(enumString);
        return retVal;
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s. enumString: %s", e.what(), enumString.c_str());
        std::cout << "Caught exception: " << e.what() << std::endl;
    }
    return worldStateFunctionEnum::INVALID;
}
*/

#endif /* CWORLDSTATEFUNCTIONTYPESFUNCTIONS_HPP_ */
