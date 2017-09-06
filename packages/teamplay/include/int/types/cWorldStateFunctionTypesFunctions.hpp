 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

typedef boost::function<bool()> worldstateFunctionType;

std::map<worldStateFunctionEnum, worldstateFunctionType> worldStateFunctionMappingEnumToFunc = boost::assign::map_list_of
            (worldStateFunctionEnum::TRUE, boost::bind(&returnTrue))
            (worldStateFunctionEnum::FALSE, boost::bind(&returnFalse))
            (worldStateFunctionEnum::IS_LOWEST_ACTIVE_ROBOT, boost::bind(&isLowestActiveRobotID))
            (worldStateFunctionEnum::IS_HIGHEST_ACTIVE_ROBOT, boost::bind(&isHighestActiveRobotID))
            (worldStateFunctionEnum::IS_SECOND_HIGHEST_ACTIVE_ROBOT, boost::bind(&isSecondHighestActiveRobotID))
            (worldStateFunctionEnum::IS_THIRD_HIGHEST_ACTIVE_ROBOT, boost::bind(&isThirdHighestActiveRobotID))
			(worldStateFunctionEnum::IS_ONLY_ACTIVE_ROBOT, boost::bind(&isOnlyActiveRobotID))
            (worldStateFunctionEnum::IS_IN_MATCH, boost::bind(&isInMatch))
            (worldStateFunctionEnum::IS_SETPIECE, boost::bind(&isSetPiece))
            (worldStateFunctionEnum::IS_OWN_SETPIECE, boost::bind(&isOwnSetPiece))
            (worldStateFunctionEnum::IS_PREPARE_SETPIECE, boost::bind(&isPrepareSetPiece))
            (worldStateFunctionEnum::IS_KICKOFF_SETPIECE, boost::bind(&isKickoffSetPiece))
            (worldStateFunctionEnum::IS_DROPPEDBALL_SETPIECE, boost::bind(&isDroppedBallSetPiece))
            (worldStateFunctionEnum::IS_SIDELINE_SETPIECE, boost::bind(&isSidelineSetPiece))
			(worldStateFunctionEnum::IS_SIDELINE_SETPIECE_RIGHT, boost::bind(&isSidelineSetPieceRight))
            (worldStateFunctionEnum::IS_PENALTY_SETPIECE, boost::bind(&isPenaltySetPiece))
            (worldStateFunctionEnum::IS_GOALKICK_SETPIECE, boost::bind(&isGoalkickSetPiece))
            (worldStateFunctionEnum::IS_FREEKICK_SETPIECE, boost::bind(&isFreekickSetPiece))
            (worldStateFunctionEnum::IS_CORNER_SETPIECE, boost::bind(&isCornerSetPiece))
            (worldStateFunctionEnum::IS_THROWIN_SETPIECE, boost::bind(&isThrowinSetPiece))
            (worldStateFunctionEnum::WITHIN_1M_OF_BALL, boost::bind(&within1mOfBall))
            (worldStateFunctionEnum::DOES_TEAM_HAVE_BALL, boost::bind(&doesTeamHaveBall))
            (worldStateFunctionEnum::DOES_OPPONENT_HAVE_BALL, boost::bind(&doesOpponentHaveBall))
            (worldStateFunctionEnum::DOES_NO_TEAM_HAVE_BALL, boost::bind(&doesNoTeamHaveBall))
            (worldStateFunctionEnum::DOES_OWN_ROBOT_HAVE_BALL, boost::bind(&doesOwnRobotHaveBall))
            (worldStateFunctionEnum::IS_OWN_ROBOT_AT_OPPONENT_SIDE, boost::bind(&isOwnRobotAtOpponentSide))
            (worldStateFunctionEnum::IS_BALL_AT_OPPONENT_SIDE, boost::bind(&isBallAtOpponentSide))
            (worldStateFunctionEnum::IS_BALL_AT_OWN_SIDE, boost::bind(&isBallAtOwnSide))
            (worldStateFunctionEnum::IS_BALL_AT_LEFT_SIDE, boost::bind(&isBallAtLeftSide))
            (worldStateFunctionEnum::IS_BALL_AT_RIGHT_SIDE, boost::bind(&isBallAtRightSide))
            (worldStateFunctionEnum::IS_OWN_ROBOT_NEAREST_TO_BALL, boost::bind(&isOwnRobotNearestToBall))
            (worldStateFunctionEnum::IS_OWN_ROBOT_NEAREST_TO_LAST_KNOWN_BALL_LOCATION, boost::bind(&isOwnRobotNearestToLastKnownBallLocation))
            (worldStateFunctionEnum::IS_BALL_LOCATION_KNOWN, boost::bind(&isBallLocationKnown))
            (worldStateFunctionEnum::IS_BALL_IN_OWN_PENALTY_AREA, boost::bind(&isBallInOwnPenaltyArea))
            (worldStateFunctionEnum::IS_BALL_IN_OPPONENT_PENALTY_AREA, boost::bind(&isBallInOpponentPenaltyArea))
            (worldStateFunctionEnum::IS_MEMBER_IN_OWN_PENALTY_AREA, boost::bind(&isMemberInOwnPenaltyArea))
            (worldStateFunctionEnum::IS_MEMBER_IN_OPPONENT_PENALTY_AREA, boost::bind(&isMemberInOpponentPenaltyArea))
            (worldStateFunctionEnum::BALL_PICKUP_ON_OPP_HALF, boost::bind(&ballPickupOnOpponentHalf))
            (worldStateFunctionEnum::IS_ASSISTENT_PRESENT, boost::bind(&isAssistentPresent))
            (worldStateFunctionEnum::IS_CLOSEST_ATTACKER_TO_BALL, boost::bind(&isClosestAttackerToBall))
            (worldStateFunctionEnum::IS_CLOSEST_DEFENDER_TO_BALL, boost::bind(&isClosestDefenderToBall))
            (worldStateFunctionEnum::IS_POTENTIAL_OPP_ATTACKER_PRESENT, boost::bind(&isPotentialOppAttackerPresent))
            (worldStateFunctionEnum::IS_SHORT_TURN_TO_GOAL_BLOCKED_BY_OPPONENT, boost::bind(&isShortTurnToGoalBlockedByOpponent))
            (worldStateFunctionEnum::IS_LONG_TURN_TO_GOAL_BLOCKED_BY_OPPONENT, boost::bind(&isLongTurnToGoalBlockedByOpponent))
            (worldStateFunctionEnum::IS_SHOT_AT_GOAL_ALLOWED, boost::bind(&isShotAtGoalAllowed))
            (worldStateFunctionEnum::IS_OPPONENT_GOALKEEPER_IN_LEFT_CORNER, boost::bind(&isOpponentGoalKeeperInLeftCorner))
            (worldStateFunctionEnum::IS_OPPONENT_GOALKEEPER_IN_RIGHT_CORNER, boost::bind(&isOpponentGoalKeeperInRightCorner))
            (worldStateFunctionEnum::IS_BALL_APPROACHING_ROBOT, boost::bind(&isBallApproachingRobot))
            (worldStateFunctionEnum::IS_OPPONENT_HALF_REACHABLE, boost::bind(&isOpponentHalfReachable))
            (worldStateFunctionEnum::IS_SHOT_ON_GOAL_BLOCKED, boost::bind(&isShotOnGoalBlocked))
            (worldStateFunctionEnum::IS_LOB_SHOT_ON_GOAL_BLOCKED, boost::bind(&isLobShotOnGoalBlocked))
            (worldStateFunctionEnum::IS_PASS_TO_CLOSEST_TEAMMEMBER_BLOCKED, boost::bind(&isPassToClosestTeammemberBlocked))
            (worldStateFunctionEnum::IS_PASS_TO_CLOSEST_ATTACKER_BLOCKED, boost::bind(&isPassToClosestAttackerBlocked))
            (worldStateFunctionEnum::IS_PASS_TO_FURTHEST_ATTACKER_BLOCKED, boost::bind(&isPassToFurthestAttackerBlocked))
            (worldStateFunctionEnum::IS_TIP_IN_BLOCKED, boost::bind(&isTipInBlocked))
            (worldStateFunctionEnum::IS_PATH_TO_BALL_BLOCKED, boost::bind(&isPathToBallBlocked))
            (worldStateFunctionEnum::DOES_ASSISTANT_HAVE_BALL, boost::bind(&doesAssistantHaveBall))
            (worldStateFunctionEnum::ALL_ROBOTS_ACTIVE, boost::bind(&allRobotsActive))
            (worldStateFunctionEnum::DEFENDING_STRATEGY_ON, boost::bind(&defendingStrategyOn))
            (worldStateFunctionEnum::MULTIPLE_OPPONENTS_ON_OWN_HALF, boost::bind(&multipleOpponentsOnOwnHalf))
            (worldStateFunctionEnum::IS_OPPONENT_WITHIN_X_METER_FROM_OWN_GOAL, boost::bind(&isOpponentWithinXMeterFromOwnGoal))
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
