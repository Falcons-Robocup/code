 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * stoppingControllerTest.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>

#include "int/stoppingController.hpp"


class AStoppingController : public ::testing::Test
{
public:
    AStoppingController()
    : gameIsStopped(false)
    {
        stoppingController.stoppedSignalSubscribe(boost::bind(&AStoppingController::stoppedSignalHandler, this));
    }

    void stoppedSignalHandler()
    {
        gameIsStopped = true;
    }

    StoppingController stoppingController;
    ArbiterGameData gameData;
    bool gameIsStopped;
};


TEST_F(AStoppingController, StopsTheGameWhenNoRobotMoves)
{
    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // One robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // The last moving robot stops moving
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_TRUE(gameIsStopped);
}


TEST_F(AStoppingController, StopsTheGameWhenTimeElapses)
{
    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // One robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // Time almost elapses
    stoppingController.control(gameData, 4.99);
    EXPECT_FALSE(gameIsStopped);

    // Time elapses
    stoppingController.control(gameData, 5.01);
    EXPECT_TRUE(gameIsStopped);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
