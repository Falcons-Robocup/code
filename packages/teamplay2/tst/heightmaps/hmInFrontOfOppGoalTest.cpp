// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmInFrontOfOppGoalTest.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmInFrontOfOppGoal.hpp"

/* SUT dependencies */
#include "int/stores/BallStore.hpp"
#include "int/stores/RobotStore.hpp"


/* Testing the 'in front of opponent goal' heightmap */

class hmInFrontOfOppGoalTest : public TeamplayTest
{
public:
    hmInFrontOfOppGoalTest()
    {
        RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::GOALKEEPER, geometry::Pose2D(0.0, 2.0, 0.0), geometry::Velocity2D()));
        BallStore::getInstance().getBall().setPosition(Point3D(1.0, 1.0, 1.0));
    }
    hmInFrontOfOppGoal _hmInFrontOfOppGoal;
};

TEST_F(hmInFrontOfOppGoalTest, onBothSides)
{
    _hmInFrontOfOppGoal.precalculate();

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
    _hmInFrontOfOppGoal.refine();
}

TEST_F(hmInFrontOfOppGoalTest, onSideWithBall)
{
    _hmInFrontOfOppGoal.precalculate();

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::ATTACKER_MAIN);
    _hmInFrontOfOppGoal.refine();
}

TEST_F(hmInFrontOfOppGoalTest, onSideWithoutBall)
{
    _hmInFrontOfOppGoal.precalculate();

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::ATTACKER_ASSIST);
    _hmInFrontOfOppGoal.refine();
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
