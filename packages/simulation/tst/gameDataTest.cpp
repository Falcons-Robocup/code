 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameDataTest.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include "int/gameDataFactory.hpp"


class AGame : public ::testing::Test
{
public:
    AGame()
    {
        gameData = GameDataFactory::createGameData(5, 5);
    }

    GameData gameData;
};


TEST_F(AGame, ConsistsOfTwoTeams)
{
    EXPECT_EQ(2, gameData.team.size());
}

TEST_F(AGame, ConsistsOfFiveRobotsPerTeam)
{
    EXPECT_EQ(5, gameData.team[TeamID::A].size());
    EXPECT_EQ(5, gameData.team[TeamID::B].size());
}

TEST_F(AGame, NoRobotHasBall)
{
    EXPECT_FALSE(gameData.anyRobotHasBall());
    EXPECT_EQ(boost::none, gameData.getTeamWithBall());
    EXPECT_EQ(boost::none, gameData.getRobotWithBall(TeamID::A));
    EXPECT_EQ(boost::none, gameData.getRobotWithBall(TeamID::B));
}

TEST_F(AGame, OneRobotHasBall)
{
    gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(-3.0, -5.0,  0.5 * M_PI));
    gameData.team[TeamID::A][RobotID::r1].setBallHandlingModulePresent();
    gameData.ball.setLocation(Point2D(-3.0, -4.8));

    EXPECT_TRUE(gameData.anyRobotHasBall());
    EXPECT_EQ(TeamID::A, gameData.getTeamWithBall());
    EXPECT_EQ(RobotID::r1, gameData.getRobotWithBall(TeamID::A));
    EXPECT_EQ(boost::none, gameData.getRobotWithBall(TeamID::B));
}

TEST_F(AGame, NoRobotIsMoving)
{
    EXPECT_FALSE(gameData.anyRobotIsMoving());
}

TEST_F(AGame, OneRobotIsMoving)
{
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    EXPECT_TRUE(gameData.anyRobotIsMoving());
}

TEST_F(AGame, DistanceOfClosestRobotToPOI)
{
    EXPECT_FLOAT_EQ(1.0, gameData.getDistanceOfClosestRobotTo(Point2D(0.0, -1.0), TeamID::A));
    EXPECT_FLOAT_EQ(3.0, gameData.getDistanceOfClosestRobotTo(Point2D(0.0, -1.0), TeamID::B));
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
