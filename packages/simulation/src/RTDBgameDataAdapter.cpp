 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * GameDataAdapter.cpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Martijn van Veen
 */

#include "int/RTDBgameDataAdapter.hpp"

#include "int/gameData.hpp"
#include "int/gameDataFactory.hpp"
#include "int/RTDBaccess.hpp"
#include "int/simulationCapabilities.hpp"

#include "FalconsRtDB2.hpp"
#include "tracing.hpp"

void RTDBgameDataAdapter::publishGameData (const GameData& gameData) const
{
    TRACE_FUNCTION("");

    ballResult ball;
    ball.confidence = 1.0;

    ball.position.x = gameData.ball.getPosition().x;
    ball.position.y = gameData.ball.getPosition().y;
    ball.position.z = gameData.ball.getPosition().z;
    ball.velocity.x = gameData.ball.getVelocity().x;
    ball.velocity.y = gameData.ball.getVelocity().y;
    ball.velocity.z = gameData.ball.getVelocity().z;

    auto teamWithBall = gameData.getTeamWithBall();
    if (teamWithBall)
    {
        auto robotWithBall = gameData.getRobotWithBall(*teamWithBall);
        if (*teamWithBall == TeamID::A)
        {
            ball.owner.type = ballPossessionTypeEnum::TEAM;
            ball.owner.robotId = getRobotNumber(*robotWithBall);
        }
        else
        {
            ball.owner.type = ballPossessionTypeEnum::OPPONENT;
        }
    }
    else /* No team has the ball */
    {
        ball.owner.type = ballPossessionTypeEnum::FIELD;
    }

    T_BALLS balls;
    balls.push_back(ball);

    auto rtdbConnection = getRTDBConnection();
    tprintf("put BALLS [0] position=[%6.2f, %6.2f, %6.2f] velocity=[%6.2f, %6.2f, %6.2f] confidence=%6.2f owner.type=%s owner.robotId=%d",
            balls[0].position.x, balls[0].position.y, balls[0].position.z,
            balls[0].velocity.x, balls[0].velocity.y, balls[0].velocity.z,
            balls[0].confidence,
            enum2str(balls[0].owner.type),
            balls[0].owner.robotId
            );
    auto r = rtdbConnection->put(BALLS, &balls);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing BALLS to RtDB");
    }

    T_OBSTACLES obstacles;
    for (auto& robotpair: gameData.team.at(TeamID::B))
    {
        auto& robot = robotpair.second;

        obstacleResult obstacle;
        obstacle.confidence = 1.0;
        obstacle.position.x = robot.getPosition().x;
        obstacle.position.y = robot.getPosition().y;
        obstacle.velocity.x = robot.getVelocity().x;
        obstacle.velocity.y = robot.getVelocity().y;

        obstacles.push_back(obstacle);
        tprintf("put OBSTACLE position=[%6.2f, %6.2f] velocity=[%6.2f, %6.2f] confidence=%6.2f id=%d",
                obstacle.position.x, obstacle.position.y,
                obstacle.velocity.x, obstacle.velocity.y,
                obstacle.confidence,
                obstacle.id
                );
    }

    r = rtdbConnection->put(OBSTACLES, &obstacles);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing OBSTACLES to RtDB");
    }
}

void RTDBgameDataAdapter::publishGameData (const GameData& gameData,
                            const TeamID teamID, const RobotID robotID) const
{
    TRACE_FUNCTION("");

    ballResult ball;
    ball.confidence = 1.0;

    ball.position.x = gameData.ball.getPositionFCS(teamID).x;
    ball.position.y = gameData.ball.getPositionFCS(teamID).y;
    ball.position.z = gameData.ball.getPositionFCS(teamID).z;
    ball.velocity.x = gameData.ball.getVelocity().x;
    ball.velocity.y = gameData.ball.getVelocity().y;
    ball.velocity.z = gameData.ball.getVelocity().z;

    auto teamWithBall = gameData.getTeamWithBall();
    if (teamWithBall)
    {
        auto robotWithBall = gameData.getRobotWithBall(*teamWithBall);
        if (*teamWithBall == TeamID::A)
        {
            ball.owner.type = ballPossessionTypeEnum::TEAM;
            ball.owner.robotId = getRobotNumber(*robotWithBall);
        }
        else
        {
            ball.owner.type = ballPossessionTypeEnum::OPPONENT;
        }
    }
    else /* No team has the ball */
    {
        ball.owner.type = ballPossessionTypeEnum::FIELD;
    }

    T_BALLS balls;
    balls.push_back(ball);

    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->put(BALLS, &balls);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing BALLS to RtDB");
    }

    auto otherTeamID = otherTeam(teamID);
    T_OBSTACLES obstacles;
    for (auto& robotpair: gameData.team.at(otherTeamID))
    {
        auto robot = robotpair.second;

        obstacleResult obstacle;
        obstacle.confidence = 1.0;
        obstacle.position.x = robot.getPosition().x;
        obstacle.position.y = robot.getPosition().y;
        obstacle.velocity.x = robot.getVelocity().x;
        obstacle.velocity.y = robot.getVelocity().y;

        obstacles.push_back(obstacle);
    }
    r = rtdbConnection->put(OBSTACLES, &obstacles);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing OBSTACLES to RtDB");
    }

    T_ROBOT_STATE robot;
    robot.status = robotStatusEnum::INPLAY;
    robot.timestamp = rtime::now();
    robot.robotId = (int)robotID;
    robot.teamId = "A"; // according to worldModel, ok to leave on A even for simulated team B thanks to database separation

    robot.position.x = gameData.team.at(teamID).at(robotID).getPositionFCS().x;
    robot.position.y = gameData.team.at(teamID).at(robotID).getPositionFCS().y;
    robot.position.Rz = gameData.team.at(teamID).at(robotID).getPositionFCS().phi;
    robot.velocity.x = gameData.team.at(teamID).at(robotID).getVelocityFCS().x;
    robot.velocity.y = gameData.team.at(teamID).at(robotID).getVelocityFCS().y;
    robot.velocity.Rz = gameData.team.at(teamID).at(robotID).getVelocityFCS().phi;

    robot.hasBall = gameData.team.at(teamID).at(robotID).canKickBall(gameData.ball.getPosition());

    if (robot.hasBall)
    {
        robot.ballAcquired.x = gameData.ball.getPickupLocation().x;
        robot.ballAcquired.y = gameData.ball.getPickupLocation().y;
    }

    tprintf("put ROBOT_STATE status=%s position=[%6.2f, %6.2f, %6.2f] velocity=[%6.2f, %6.2f, %6.2f] hasBall=%s ballAcquired=[%6.2f, %6.2f]",
            enum2str(robot.status),
            robot.position.x, robot.position.y, robot.position.Rz,
            robot.velocity.x, robot.velocity.y, robot.velocity.Rz,
            ((robot.hasBall)?("Yes"):("No ")),
            robot.ballAcquired.x, robot.ballAcquired.y
            );
    r = rtdbConnection->put(ROBOT_STATE, &robot);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing ROBOT_STATE to RtDB");
    }
}
