 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * runningControllerTest.cpp
 *
 *  Created on: Dec 29, 2018
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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
    EXPECT_EQ(Setpiece::KICKOFF, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::B, lastJudgement.teamID);
    EXPECT_EQ(1, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


TEST_F(ARunningController, BallIsBehindPositiveGoallineLastTouchedByOffendingTeam)
{
    gameData.ball.setLocation(Point2D(4.0, 11.1));
    gameData.teamLastHoldingBall = TeamID::A;

    runningController.control(gameData);

    EXPECT_EQ(Point2D(3.0, 7.0), lastJudgement.ballPosition);
    EXPECT_EQ(Setpiece::GOALKICK, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::B, lastJudgement.teamID);
    EXPECT_EQ(0, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


TEST_F(ARunningController, BallIsBehindPositiveGoallineLastTouchedByDefendingTeam)
{
    gameData.ball.setLocation(Point2D(4.0, 11.1));
    gameData.teamLastHoldingBall = TeamID::B;

    runningController.control(gameData);

    EXPECT_EQ(Point2D(5.5, 8.5), lastJudgement.ballPosition);
    EXPECT_EQ(Setpiece::CORNER, lastJudgement.setpiece);
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
    EXPECT_EQ(Setpiece::THROWIN, lastJudgement.setpiece);
    EXPECT_EQ(TeamID::A, lastJudgement.teamID);
    EXPECT_EQ(0, score_team_A);
    EXPECT_EQ(0, score_team_B);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
