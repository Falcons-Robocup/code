 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionTypes.hpp
 *
 *  Created on: Apr 30, 2016
 *      Author: Erik Kouters
 */

#ifndef CACTIONTYPES_HPP_
#define CACTIONTYPES_HPP_

#include <string>
#include <map>
#include <boost/assign/list_of.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/assign/ptr_map_inserter.hpp>
#include "int/actions/cAbstractAction.hpp"
#include "int/types/cActionEnumTypes.hpp"

extern boost::ptr_map<actionEnum, boost::shared_ptr<cAbstractAction> > enumToActionMapping;

static std::map<std::string, actionEnum> actionMapping = boost::assign::map_list_of
        ("invalid", actionEnum::INVALID)
        ("success", actionEnum::SUCCESS)
        ("moveWhileTurning", actionEnum::MOVE_WHILE_TURNING)
        ("move", actionEnum::MOVE)
        ("stop", actionEnum::STOP)
        ("shoot", actionEnum::SHOOT)
        ("turn", actionEnum::TURN)
        ("positionBeforePOI", actionEnum::POSITION_BEFORE_POI)
        ("positionBehindPOI", actionEnum::POSITION_BEHIND_POI)
        ("faceNearestTeammember", actionEnum::FACE_NEAREST_TEAMMEMBER)
		("getBall", actionEnum::GET_BALL)
        ("goalKeeper", actionEnum::GOALKEEPER)
		("moveToFreeSpot", actionEnum::MOVE_TO_FREE_SPOT)
        ("moveToPenaltyAngle", actionEnum::MOVE_TO_PENALTY_ANGLE)
        ("interceptBall", actionEnum::INTERCEPT_BALL)
		("avoidPOI", actionEnum::AVOID_POI)
		("getBallOnVector", actionEnum::GET_BALL_ON_VECTOR)
		("longTurnToGoal", actionEnum::LONG_TURN_TO_GOAL)
		("aimForShotOnGoal", actionEnum::AIM_FOR_SHOT_ON_GOAL)
		("defendAssist", actionEnum::DEFEND_ASSIST)

        ;

enum class shootEnum
{
    INVALID = 0,

    PASS_TOWARDS_NEAREST_TEAMMEMBER,
	PASS_TOWARDS_NEAREST_ATTACKER,
	PASS_TOWARDS_FURTHEST_ATTACKER,
	PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF,
	PASS_TOWARDS_TIP_IN_POSITION,
    SHOOT_TOWARDS_GOAL,
	PASS_TOWARDS_GOALIE,
	LOB_TOWARDS_GOAL,

    SIZE_OF_ENUM
};

static std::map<std::string, shootEnum> shootMapping = boost::assign::map_list_of
        ("invalid", shootEnum::INVALID)
        ("passTowardsNearestTeammember", shootEnum::PASS_TOWARDS_NEAREST_TEAMMEMBER)
		("passTowardsNearestAttacker", shootEnum::PASS_TOWARDS_NEAREST_ATTACKER)
		("passTowardsFurthestAttacker", shootEnum::PASS_TOWARDS_FURTHEST_ATTACKER)
		("passTowardsNearestAttackerOnOppHalf", shootEnum::PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF)
		("passTowardsTipInPosition", shootEnum::PASS_TOWARDS_TIP_IN_POSITION)
		("shootTowardsGoal", shootEnum::SHOOT_TOWARDS_GOAL)
		("passToGoalie", shootEnum::PASS_TOWARDS_GOALIE)
		("lobTowardsGoal", shootEnum::LOB_TOWARDS_GOAL)
        ;

#endif /* CACTIONTYPES_HPP_ */
