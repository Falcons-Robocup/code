// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
