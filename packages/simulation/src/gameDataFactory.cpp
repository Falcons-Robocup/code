 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameDataFactory.cpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#include "int/gameDataFactory.hpp"

GameData GameDataFactory::createGameData(const int sizeTeamA, const int sizeTeamB)
{
    GameData gameData;

    /* Construct two teams of five robots each */
    gameData.team =
    {
        {TeamID::A,
            {
                {RobotID::r1, Robot()},
                {RobotID::r2, Robot()},
                {RobotID::r3, Robot()},
                {RobotID::r4, Robot()},
                {RobotID::r5, Robot()}
            }
        },
        {TeamID::B,
            {
                {RobotID::r1, Robot()},
                {RobotID::r2, Robot()},
                {RobotID::r3, Robot()},
                {RobotID::r4, Robot()},
                {RobotID::r5, Robot()}
            }
        }
    };

    /* Remove superfluous robots */
    auto it = gameData.team[TeamID::A].begin();
    for (int i = 0; i < sizeTeamA; i++) { it++; }
    gameData.team[TeamID::A].erase(it, gameData.team[TeamID::A].end());

    /* Set robot properties: location, playing direction, ... */
    double x = -4.0;
    for (auto& robotpair: gameData.team[TeamID::A])
    {
        const auto& robotID = robotpair.first;
        auto& robot = robotpair.second;
        robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
        robot.setPosition(Position2D(x, -2.0, 0.5 * M_PI));
        x += 2.0;

        if (robotID == RobotID::r1)
        {
            robot.setBallHandlingModuleAbsent();
        }
        else
        {
            robot.setBallHandlingModulePresent();
        }
    }

    it = gameData.team[TeamID::B].begin();
    for (int i = 0; i < sizeTeamB; i++) { it++; }
    gameData.team[TeamID::B].erase(it, gameData.team[TeamID::B].end());

    x = -4.0;
    for (auto& robotpair: gameData.team[TeamID::B])
    {
        const auto& robotID = robotpair.first;
        auto& robot = robotpair.second;
        robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
        robot.setPosition(Position2D(x,  2.0, 1.5 * M_PI));
        x += 2.0;

        if (robotID == RobotID::r1)
        {
            robot.setBallHandlingModuleAbsent();
        }
        else
        {
            robot.setBallHandlingModulePresent();
        }
    }

    return gameData;
}
