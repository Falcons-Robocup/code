// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 *  cQueryInterfaceTest.cpp
 *
 *  Created on: Apr 24, 2018
 *      Author: Erik Kouters / Coen Tempelaars
 */

/* Include testframework */
#include "gtest/gtest.h"
#include "gmock/gmock.h"

/* SUT */
#include "int/cQueryInterface.hpp"

/* SUT dependencies */


class cQueryInterfaceTest : public ::testing::Test
{
    public:
        cQueryInterfaceTest()
        {
            // Connect WorldModelInterface to cQueryInterface
            MP_WorldModelInterface wm;
            _queryInterface.connect(&wm);

            // Set data in WorldModelInterface
            // NOTE: wm.setData is now protected, but this should probably be moved to public.
            //       And it has too many parameters -- split into multiple setters.
            // TODO
            // wm.setData()
        };

        cQueryInterface _queryInterface;
};

/* test:
 * - timeToBall for own robot
 * - timeToBall for teammember
 */
TEST_F(cQueryInterfaceTest, timeToBallOwnRobot)
{
    // TODO
    EXPECT_EQ(1,1);
}
TEST_F(cQueryInterfaceTest, timeToBallTeammember)
{
    // TODO
    EXPECT_EQ(1,1);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
