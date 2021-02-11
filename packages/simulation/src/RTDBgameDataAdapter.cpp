// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include "cEnvironmentField.hpp"
#include <mutex>
#include "ftime.hpp"


std::mutex g_mutex_sim;


RTDBgameDataAdapter::RTDBgameDataAdapter()
{
    // Start a thread to monitor SIMULATION_SCENE, in case user modifies it
    _sceneMonitorThread = std::thread(&RTDBgameDataAdapter::monitorScene, this);
}

RTDBgameDataAdapter::~RTDBgameDataAdapter()
{
    // Stop the thread. This causes Signal 6 (SIGABRT)
    _sceneMonitorThread.std::thread::~thread();
}

void addNonRobotObstaclesTo(std::vector<Obstacle> const &nonRobotObstacles, T_OBSTACLES &resultObstacles)
{
    for (auto& obst: nonRobotObstacles)
    {
        obstacleResult obstacle;
        obstacle.confidence = 1.0;
        obstacle.position.x = obst.position.x;
        obstacle.position.y = obst.position.y;
        obstacle.velocity.x = obst.velocity.x;
        obstacle.velocity.y = obst.velocity.y;
        resultObstacles.push_back(obstacle);
    }
}

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
            ball.owner.robotId = 0;
        }
    }
    else /* No team has the ball */
    {
        ball.owner.type = ballPossessionTypeEnum::FIELD;
        ball.owner.robotId = 0;
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
    addNonRobotObstaclesTo(gameData.nonRobotObstacles, obstacles);
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
            ball.owner.robotId = 0;
        }
    }
    else /* No team has the ball */
    {
        ball.owner.type = ballPossessionTypeEnum::FIELD;
        ball.owner.robotId = 0;
    }

    T_BALLS balls;
    // hide ball if far outside field
    if (abs(ball.position.x) < (0.5 * cEnvironmentField::getInstance().getWidth() + 2.0) &&
        abs(ball.position.y) < (0.5 * cEnvironmentField::getInstance().getLength() + 2.0))
    {
        balls.push_back(ball);
    }

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
    addNonRobotObstaclesTo(gameData.nonRobotObstacles, obstacles);
    r = rtdbConnection->put(OBSTACLES, &obstacles);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing OBSTACLES to RtDB");
    }

    T_ROBOT_STATE robot;
    robot.status = robotStatusEnum::INPLAY;
    robot.timestamp = ftime::now();
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

void RTDBgameDataAdapter::publishScene(const GameData& gameData) const
{
    TRACE_FUNCTION("");

    // convert to SimulationScene
    SimulationScene scene;
    auto now = ftime::now();

    // robots and opponents
    for (auto& teampair: gameData.team)
    {
        const auto teamID = teampair.first;
        auto &targetRobots = (teamID == TeamID::A) ? scene.robots : scene.opponents;
        for (auto& robotpair: gameData.team.at(teamID))
        {
            SimulationSceneRobot r;
            r.robotId = 1 + (int)robotpair.first;
            Position2D pos = robotpair.second.getPosition();
            r.position = pose(pos.x, pos.y, pos.phi);
            Velocity2D vel = robotpair.second.getVelocity();
            r.velocity = pose(vel.x, vel.y, vel.phi);
            targetRobots.push_back(r);
        }
    }

    // ball
    {
        SimulationSceneBall b;
        Point3D pos = gameData.ball.getPosition();
        Vector3D vel = gameData.ball.getVelocity();
        b.position.x = pos.x;
        b.position.y = pos.y;
        b.position.z = pos.z; // TODO #14
        b.velocity.x = vel.x;
        b.velocity.y = vel.y;
        b.velocity.z = vel.z;
        scene.balls.push_back(b);
    }

    // there are never any extra obstacles at simworld initializatoin

    // write to RTDB
    auto rtdbConnection = getRTDBConnection();
    rtdbConnection->put(SIMULATION_SCENE, &scene);
}

bool RTDBgameDataAdapter::checkUpdatedScene(SimulationScene &scene)
{
    TRACE_FUNCTION("");
    // interface to client (simworld): to notify new scene
    std::lock_guard<std::mutex> l(g_mutex_sim);
    if (_sceneUpdated)
    {
        _sceneUpdated = false; // reset flag
        auto rtdbConnection = getRTDBConnection();
        SimulationScene tmpScene;
        int r = rtdbConnection->get(SIMULATION_SCENE, &tmpScene);
        TRACE("r=%d", r);
        tprintf("got new SimulationScene, r=%d", r);
        if (r == RTDB2_SUCCESS)
        {
            scene = tmpScene;
            return true;
        }
    }
    return false;
}

void RTDBgameDataAdapter::monitorScene()
{
    TRACE_FUNCTION("");

    // Prevent race condition with other init time RTDB connections being created
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (true)
    {
        auto rtdbConnection = getRTDBConnection();
        rtdbConnection->waitForPut(SIMULATION_SCENE);
        {
            std::lock_guard<std::mutex> l(g_mutex_sim);
            // raise a flag to signal main thread
            tprintf("raising the flag");
            _sceneUpdated = true;
        }
    }
}

