// Copyright 2018-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * runningControllerTest.cpp
 *
 *  Created on: Dec 29, 2018
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "cEnvironmentField.hpp"
#include "int/runningController.hpp"


class ARunningController : public ::testing::Test
{
public:
    ARunningController()
    : score_team_A(0)
    , score_team_B(0)
    {
        runningController.goalSignalSubscribe(boost::bind(&ARunningController::goalSignalHandler, this, _1));
        runningController.stoppingSignalSubscribe(boost::bind(&ARunningController::stoppingSignalHandler, this, _1));
    }

    void goalSignalHandler(const TeamID& teamID)
    {
        if (teamID == TeamID::A)
        {
            score_team_A += 1;
        }
        else
        {
            score_team_B += 1;
        }
    }

    void stoppingSignalHandler(const Judgement judgement)
    {
        lastJudgement = judgement;
    }

    RunningController runningController;
    ArbiterGameData gameData;
    Judgement lastJudgement;
    int score_team_A;
    int score_team_B;
};


TEST_F(ARunningController, BallIsInPositiveGoal)
{
    gameData.ball.setLocation(Point2D(0.0, 11.1));

    runningController.control(gameData);

    EXPECT_EQ(Point2D(0.0, 0.0), lastJudgement.ballPosition);
    EXPECT_EQ(SetpieceEnum::KICKOFF, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::B, lastJudgement.teamID);
    EXPECT_EQ(1, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


TEST_F(ARunningController, BallIsBehindPositiveGoallineLastTouchedByOffendingTeam)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    gameData.ball.setLocation(Point2D(4.0, 11.1));
    gameData.teamLastHoldingBall = TeamID::A;

    runningController.control(gameData);

    EXPECT_EQ(Point2D(3.0, 7.0), lastJudgement.ballPosition);
    EXPECT_EQ(SetpieceEnum::GOALKICK, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::B, lastJudgement.teamID);
    EXPECT_EQ(0, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


TEST_F(ARunningController, BallIsBehindPositiveGoallineLastTouchedByDefendingTeam)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    gameData.ball.setLocation(Point2D(4.0, 11.1));
    gameData.teamLastHoldingBall = TeamID::B;

    runningController.control(gameData);

    EXPECT_EQ(Point2D(5.5, 8.5), lastJudgement.ballPosition);
    EXPECT_EQ(SetpieceEnum::CORNER, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::A, lastJudgement.teamID);
    EXPECT_EQ(0, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


TEST_F(ARunningController, BallIsAcrossSideline)
{
    gameData.ball.setLocation(Point2D(7.1, 3.0));
    gameData.teamLastHoldingBall = TeamID::B;

    runningController.control(gameData);

    EXPECT_NEAR(5.7, lastJudgement.ballPosition.x, 0.1);
    EXPECT_NEAR(2.4, lastJudgement.ballPosition.y, 0.1);
    EXPECT_EQ(SetpieceEnum::THROWIN, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::A, lastJudgement.teamID);
    EXPECT_EQ(0, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
