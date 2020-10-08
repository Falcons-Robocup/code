 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * preparingControllerTest.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/preparingController.hpp"


class APreparingController : public ::testing::Test
{
public:
    APreparingController()
    : gameIsPrepared(false)
    , gameIsStopping(false)
    {
        preparingController.preparedSignalSubscribe(boost::bind(&APreparingController::preparedSignalHandler, this));
        preparingController.stoppingSignalSubscribe(boost::bind(&APreparingController::stoppingSignalHandler, this));
    }

    void preparedSignalHandler()
    {
        gameIsPrepared = true;
    }

    void stoppingSignalHandler()
    {
        gameIsStopping = true;
    }

    PreparingController preparingController;
    ArbiterGameData gameData;
    bool gameIsPrepared;
    bool gameIsStopping;
};


TEST_F(APreparingController, DeclaresTheGamePreparedWhenNoRobotMoves)
{
    // Initially no robot moves, but that does not mean the game is prepared
    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // After one second, one robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));
    preparingController.control(gameData, 1.0);
    EXPECT_FALSE(gameIsPrepared);

    // After another second, the last moving robot stops moving
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(0.0, 0.0, 0.0));
    preparingController.control(gameData, 2.0);
    EXPECT_TRUE(gameIsPrepared);
    EXPECT_FALSE(gameIsStopping);
}


TEST_F(APreparingController, DeclaresTheGamePreparedWhenTimeElapses)
{
    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // One robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // Time almost elapses
    preparingController.control(gameData, 9.99);
    EXPECT_FALSE(gameIsPrepared);

    // Time elapses
    preparingController.control(gameData, 10.01);
    EXPECT_TRUE(gameIsPrepared);
    EXPECT_FALSE(gameIsStopping);
}


TEST_F(APreparingController, StopsTheGameWhenTheBallMoves)
{
    // Two robots are moving, but the ball is not
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.ball.setVelocity(Vector3D());

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);
    EXPECT_FALSE(gameIsStopping);

    // The ball starts moving slightly
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);
    EXPECT_TRUE(gameIsStopping);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
