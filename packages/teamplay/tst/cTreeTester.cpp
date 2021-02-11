// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTreeTester.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Tim Kouters
 */

#include <gtest/gtest.h>
#include "tracing.hpp"

#include "int/actions/cActionStop.hpp"
#include "int/actions/cActionShoot.hpp"
#include "int/actions/cActionPass.hpp"
#include "int/actions/cActionPositionBeforePOI.hpp"
#include "int/actions/cActionPositionBehindPOI.hpp"
#include "int/actions/cActionPositionForOppSetpiece.hpp"
#include "int/actions/cActionPositionForOwnSetpiece.hpp"
#include "int/actions/cActionGetBall.hpp"
#include "int/actions/cActionGoalKeeper.hpp"
#include "int/actions/cActionInterceptBall.hpp"
#include "int/actions/cActionMove.hpp"
#include "int/actions/cActionMoveToFreeSpot.hpp"
#include "int/actions/cActionSuccess.hpp"
#include "int/actions/cActionAvoidPOI.hpp"
#include "int/actions/cActionDefendPenaltyArea.hpp"
#include "int/actions/cActionTurnAwayFromOpponent.hpp"
#include "int/actions/cActionDefendAttackingOpponent.hpp"
#include "int/actions/cActionDribble.hpp"

// SUT
#include "int/cDecisionTree.hpp"

TEST(TreeTester, LoadAllDefaultTrees)
{
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();
    EXPECT_NO_THROW(cDecisionTree::getInstance().loadDecisionTrees(""));
}

TEST(TreeTester, LoadAllExampleTrees)
{
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();
    EXPECT_NO_THROW(cDecisionTree::getInstance().loadDecisionTrees("examples/"));
}

int main(int argc, char **argv)
{
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    boost::assign::ptr_map_insert( enumToActionMapping )
    ( tpActionEnum::INVALID, boost::make_shared<cActionStop>() )
    ( tpActionEnum::SUCCESS, boost::make_shared<cActionSuccess>() )
    ( tpActionEnum::MOVE, boost::make_shared<cActionMove>() )
    ( tpActionEnum::STOP, boost::make_shared<cActionStop>() )
    ( tpActionEnum::SHOOT, boost::make_shared<cActionShoot>() )
    ( tpActionEnum::PASS, boost::make_shared<cActionPass>() )
    ( tpActionEnum::POSITION_BEFORE_POI, boost::make_shared<cActionPositionBeforePOI>() )
    ( tpActionEnum::POSITION_BEHIND_POI, boost::make_shared<cActionPositionBehindPOI>() )
    ( tpActionEnum::POSITION_FOR_OPP_SETPIECE, boost::make_shared<cActionPositionForOppSetpiece>() )
    ( tpActionEnum::POSITION_FOR_OWN_SETPIECE, boost::make_shared<cActionPositionForOwnSetpiece>() )
    ( tpActionEnum::GET_BALL, boost::make_shared<cActionGetBall>() )
    ( tpActionEnum::GOALKEEPER, boost::make_shared<cActionGoalKeeper>() )
    ( tpActionEnum::INTERCEPT_BALL, boost::make_shared<cActionInterceptBall>() )
    ( tpActionEnum::MOVE_TO_FREE_SPOT, boost::make_shared<cActionMoveToFreeSpot>() )
    ( tpActionEnum::AVOID_POI, boost::make_shared<cActionAvoidPOI>() )
    ( tpActionEnum::DEFEND_PENALTY_AREA, boost::make_shared<cActionDefendPenaltyArea>() )
    ( tpActionEnum::TURN_AWAY_FROM_OPPONENT, boost::make_shared<cActionTurnAwayFromOpponent>() )
    ( tpActionEnum::DEFEND_ATTACKING_OPPONENT,  boost::make_shared<cActionDefendAttackingOpponent>() )
    ( tpActionEnum::DRIBBLE,  boost::make_shared<cActionDribble>())
    ;

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
