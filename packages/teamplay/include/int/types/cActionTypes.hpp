 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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
#include "tpActionEnum.hpp" // sharedTypes

extern boost::ptr_map<tpActionEnum, boost::shared_ptr<cAbstractAction> > enumToActionMapping;

static std::map<std::string, tpActionEnum> actionMapping = boost::assign::map_list_of
        ("invalid", tpActionEnum::INVALID)
        ("success", tpActionEnum::SUCCESS)
        ("move", tpActionEnum::MOVE)
        ("stop", tpActionEnum::STOP)
        ("shoot", tpActionEnum::SHOOT)
        ("pass", tpActionEnum::PASS)
        ("positionBeforePOI", tpActionEnum::POSITION_BEFORE_POI)
        ("positionBehindPOI", tpActionEnum::POSITION_BEHIND_POI)
        ("positionForOppSetpiece", tpActionEnum::POSITION_FOR_OPP_SETPIECE)
        ("positionForOwnSetpiece", tpActionEnum::POSITION_FOR_OWN_SETPIECE)
        ("getBall", tpActionEnum::GET_BALL)
        ("goalKeeper", tpActionEnum::GOALKEEPER)
        ("moveToFreeSpot", tpActionEnum::MOVE_TO_FREE_SPOT)
        ("interceptBall", tpActionEnum::INTERCEPT_BALL)
        ("avoidPOI", tpActionEnum::AVOID_POI)
        ("defendPenaltyArea", tpActionEnum::DEFEND_PENALTY_AREA)
        ("turnAwayFromOpponent", tpActionEnum::TURN_AWAY_FROM_OPPONENT)
        ("defendAttackingOpponent", tpActionEnum::DEFEND_ATTACKING_OPPONENT)
        ("dribble", tpActionEnum::DRIBBLE)

        ;

enum class shootEnum
{
    INVALID = 0,

    PASS_TOWARDS_NEAREST_TEAMMEMBER,
    PASS_TOWARDS_NEAREST_ATTACKER,
    PASS_TOWARDS_FURTHEST_ATTACKER,
    PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF,
    PASS_TOWARDS_FURTHEST_DEFENDER,
    PASS_TOWARDS_TIP_IN_POSITION,
    SHOOT_TOWARDS_GOAL,
    LOB_TOWARDS_GOAL,

    SIZE_OF_ENUM
};

static std::map<std::string, shootEnum> shootMapping = boost::assign::map_list_of
        ("invalid", shootEnum::INVALID)
        ("passTowardsNearestTeammember", shootEnum::PASS_TOWARDS_NEAREST_TEAMMEMBER)
        ("passTowardsNearestAttacker", shootEnum::PASS_TOWARDS_NEAREST_ATTACKER)
        ("passTowardsFurthestAttacker", shootEnum::PASS_TOWARDS_FURTHEST_ATTACKER)
        ("passTowardsNearestAttackerOnOppHalf", shootEnum::PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF)
        ("passTowardsFurthestDefender", shootEnum::PASS_TOWARDS_FURTHEST_DEFENDER)
        ("passTowardsTipInPosition", shootEnum::PASS_TOWARDS_TIP_IN_POSITION)
        ("shootTowardsGoal", shootEnum::SHOOT_TOWARDS_GOAL)
        ("lobTowardsGoal", shootEnum::LOB_TOWARDS_GOAL)
        ;

#endif /* CACTIONTYPES_HPP_ */
