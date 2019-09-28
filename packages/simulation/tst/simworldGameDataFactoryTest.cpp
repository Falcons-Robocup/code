 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * simworldGameDataFactoryTest.cpp
 *
 *  Created on: March 26, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/simworldGameDataFactory.hpp"


class TwoTeamsOfTwoRobots : public ::testing::Test
{
public:
    TwoTeamsOfTwoRobots()
    {
        gameData.ball.setLocation(Point2D(1.2, 3.4));
        gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 0.0));
        gameData.team[TeamID::A][RobotID::r2].setPosition(Position2D(2.0, 3.0, 0.0));
        gameData.team[TeamID::B][RobotID::r1].setPosition(Position2D(3.0, 4.0, 0.0));
        gameData.team[TeamID::B][RobotID::r2].setPosition(Position2D(4.0, 5.0, 0.0));
    }

    GameData gameData;
};


TEST_F(TwoTeamsOfTwoRobots, TransformedIntoCompleteWorld)
{
    auto simworldGameData = SimworldGameDataFactory::createCompleteWorld(gameData);
    EXPECT_EQ(2, simworldGameData.team.size());
    EXPECT_EQ(2, simworldGameData.team[TeamID::A].size());
    EXPECT_EQ(2, simworldGameData.team[TeamID::B].size());

    EXPECT_NEAR(2.0, gameData.team[TeamID::A][RobotID::r2].getPosition().x, 0.01);
    EXPECT_NEAR(5.0, gameData.team[TeamID::B][RobotID::r2].getPosition().y, 0.01);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
