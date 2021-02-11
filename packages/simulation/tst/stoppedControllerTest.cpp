// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * stoppedControllerTest.cpp
 *
 *  Created on: Jan 9, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/stoppedController.hpp"


class AStoppedController : public ::testing::Test
{
public:
    AStoppedController()
    : gameIsPrepared(false)
    {
        stoppedController.preparingSignalSubscribe(boost::bind(&AStoppedController::preparingSignalHandler, this));
    }

    void preparingSignalHandler()
    {
        gameIsPrepared = true;
    }

    StoppedController stoppedController;
    ArbiterGameData gameData;
    bool gameIsPrepared;
};


TEST_F(AStoppedController, AlwaysRaisesThePreparingSignal)
{
    EXPECT_FALSE(gameIsPrepared);
    stoppedController.control(gameData);
    EXPECT_TRUE(gameIsPrepared);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
