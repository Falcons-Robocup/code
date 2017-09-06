 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTreeTester.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Tim Kouters
 */

#include <gtest/gtest.h>
#include "int/utilities/trace.hpp"

#include "int/actions/cActionStop.hpp"
#include "int/actions/cActionShoot.hpp"
#include "int/actions/cActionPositionBeforePOI.hpp"
#include "int/actions/cActionPositionBehindPOI.hpp"
#include "int/actions/cActionFaceNearestTeammember.hpp"
#include "int/actions/cActionGetBall.hpp"
#include "int/actions/cActionGoalKeeper.hpp"
#include "int/actions/cActionInterceptBall.hpp"
#include "int/actions/cActionMove.hpp"
#include "int/actions/cActionMoveToPenaltyAngle.hpp"
#include "int/actions/cActionMoveToFreeSpot.hpp"
#include "int/actions/cActionSuccess.hpp"
#include "int/actions/cActionAvoidPOI.hpp"
#include "int/actions/cActionGetBallOnVector.hpp"
#include "int/actions/cActionLongTurnToGoal.hpp"
#include "int/actions/cActionAimForShotOnGoal.hpp"
#include "int/actions/cActionDefendAssist.hpp"
#include "int/utilities/trace.hpp"

#include "mocks/worldModelUpdated.hpp"

// SUT
#include "int/cDecisionTree.hpp"

TEST(TreeTester, LoadAllTrees)
{
	teamplay::traceRedirect::getInstance().setAllTracesToStdout();
	EXPECT_NO_THROW(cDecisionTree::getInstance());
}

int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    ros::init(argc, argv, "cTreeTesterTest");

    boost::assign::ptr_map_insert( enumToActionMapping )
        ( actionEnum::INVALID, boost::make_shared<cActionStop>() )
        ( actionEnum::SUCCESS, boost::make_shared<cActionSuccess>() )
        ( actionEnum::MOVE_WHILE_TURNING, boost::make_shared<cActionMove>() )
        ( actionEnum::MOVE, boost::make_shared<cActionMove>() )
		( actionEnum::MOVE_TO_PENALTY_ANGLE, boost::make_shared<cActionMoveToPenaltyAngle>() )
        ( actionEnum::STOP, boost::make_shared<cActionStop>() )
        ( actionEnum::SHOOT, boost::make_shared<cActionShoot>() )
        ( actionEnum::TURN, boost::make_shared<cActionMove>() )
        ( actionEnum::POSITION_BEFORE_POI, boost::make_shared<cActionPositionBeforePOI>() )
        ( actionEnum::POSITION_BEHIND_POI, boost::make_shared<cActionPositionBehindPOI>() )
        ( actionEnum::FACE_NEAREST_TEAMMEMBER, boost::make_shared<cActionFaceNearestTeammember>() )
        ( actionEnum::GET_BALL, boost::make_shared<cActionGetBall>() )
        ( actionEnum::GOALKEEPER, boost::make_shared<cActionGoalKeeper>() )
        ( actionEnum::INTERCEPT_BALL, boost::make_shared<cActionInterceptBall>() )
        ( actionEnum::MOVE_TO_FREE_SPOT, boost::make_shared<cActionMoveToFreeSpot>() )
		( actionEnum::AVOID_POI, boost::make_shared<cActionAvoidPOI>() )
		( actionEnum::GET_BALL_ON_VECTOR, boost::make_shared<cActionGetBallOnVector>() )
		( actionEnum::LONG_TURN_TO_GOAL, boost::make_shared<cActionLongTurnToGoal>() )
		( actionEnum::AIM_FOR_SHOT_ON_GOAL, boost::make_shared<cActionAimForShotOnGoal>() )
		( actionEnum::DEFEND_ASSIST, boost::make_shared<cActionDefendAssist>() )
        ;

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
