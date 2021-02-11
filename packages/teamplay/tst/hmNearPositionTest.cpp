// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearPosition.cpp
 *
 *  Created on: June 5, 2018
 *      Author: Jan Feitsma
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmNearPosition.hpp"

/* SUT dependencies */
#include "int/stores/robotStore.hpp"


/* Testing the 'near position' heightmap */

class hmNearPositionTest : public TeamplayTest
{
public:
    hmNearPositionTest()
    {
    }
    hmNearPosition _hmNearPosition;
    parameterMap_t _parameters;
};

TEST_F(hmNearPositionTest, functionalTest)
{
    _hmNearPosition.precalculate();
    
    _parameters["positionX"] = "1.0";
    _parameters["positionY"] = "6.0";
    _hmNearPosition.refine(_parameters);
    
    auto optimum = _hmNearPosition.getOptimum();

    EXPECT_EQ(1.0, optimum._center.x);
    EXPECT_EQ(6.0, optimum._center.y);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
