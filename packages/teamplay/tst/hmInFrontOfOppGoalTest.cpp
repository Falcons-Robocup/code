// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmInFrontOfOppGoalTest.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmInFrontOfOppGoal.hpp"

/* SUT dependencies */
#include "int/stores/ballStore.hpp"


/* Testing the 'in front of opponent goal' heightmap */

class hmInFrontOfOppGoalTest : public TeamplayTest
{
public:
    hmInFrontOfOppGoalTest()
    {
        ballStore::getInstance().getBall().setPosition(Point3D(1.0, 1.0, 1.0));
    }
    hmInFrontOfOppGoal _hmInFrontOfOppGoal;
    parameterMap_t _parameters;
};

TEST_F(hmInFrontOfOppGoalTest, onBothSides)
{
    _hmInFrontOfOppGoal.precalculate();

    _parameters["onSide"] = "both";
    _hmInFrontOfOppGoal.refine(_parameters);
}

TEST_F(hmInFrontOfOppGoalTest, onSideWithBall)
{
    _hmInFrontOfOppGoal.precalculate();

    _parameters["onSide"] = "withBall";
    _hmInFrontOfOppGoal.refine(_parameters);
}

TEST_F(hmInFrontOfOppGoalTest, onSideWithoutBall)
{
    _hmInFrontOfOppGoal.precalculate();

    _parameters["onSide"] = "withoutBall";
    _hmInFrontOfOppGoal.refine(_parameters);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
