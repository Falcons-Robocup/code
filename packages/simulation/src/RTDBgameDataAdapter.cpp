// Copyright 2019-2022 Martijn (Falcons)
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
#include "int/simulation_generated_enum2str.hpp" // for enums of this package (simulation)
#include "int/RTDBaccess.hpp"
#include "int/simulationCapabilities.hpp"

#include "FalconsRTDB.hpp"
#include "generated_enum2str.hpp" // for enums of facilities package
#include "tracing.hpp"
#include "cEnvironmentField.hpp"
#include <mutex>
#include "ftime.hpp"
#include <map>


std::mutex g_mutex_sim;

class VecAdapter {
public:
    VecAdapter(float x_factor, float y_factor) :
        _x_factor(x_factor),
        _y_factor(y_factor)
    {}

    void adapt(vec2d& v) const
    {
        v.x *= _x_factor;
        v.y *= _y_factor;
    }

    void adapt(vec3d& v) const
    {
        v.x *= _x_factor;
        v.y *= _y_factor;
    }

    template<typename T>
    void adapt(vec2d T::*pVec, std::vector<T>& list) const
    {
        for (T& t : list)
        {
            adapt(t.*pVec);
        }
    }

    template<typename T>
    void adapt(vec3d T::*pVec, std::vector<T>& list) const 
    {
        for (T& t : list)
        {
            adapt(t.*pVec);
        }
    }

private:
    float _x_factor;
    float _y_factor;
};

std::map<PlayingDirection, const VecAdapter> g_team_vec_adapter = {
    {PlayingDirection::LEFT_TO_RIGHT, VecAdapter( 1, 1)},
    {PlayingDirection::RIGHT_TO_LEFT, VecAdapter(-1,-1)}
};


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
        obstacle.position = convert_xy(obst.position);
        obstacle.velocity = convert_xy(obst.velocity);
        resultObstacles.push_back(obstacle);
    }
}

void RTDBgameDataAdapter::publishGameData (const GameData& gameData) const
{
    TRACE_FUNCTION("");

    ballResult ball;
    ball.confidence = 1.0;

    ball.position = convert_xyz(gameData.ball.getPosition());
    ball.velocity = convert_xyz(gameData.ball.getVelocity());

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
        obstacle.position = convert_xy(robot.getPosition());
        obstacle.velocity = convert_xy(robot.getVelocity());

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
    TRACE_FUNCTION("team = %s, robot = %s", enum2str(teamID), enum2str(robotID));
    
    PlayingDirection playingDirection = gameData.team.at(teamID).at(robotID).getPlayingDirection();
    const VecAdapter& vecAdapter = g_team_vec_adapter.at(playingDirection);

    ballResult ball;
    ball.confidence = 1.0;
    ball.position = convert_xyz(gameData.ball.getPosition()); 
    ball.velocity = convert_xyz(gameData.ball.getVelocity());
    ball.owner.robotId = 0;

    auto teamWithBall = gameData.getTeamWithBall();
    if (teamWithBall)
    {
        auto robotWithBall = gameData.getRobotWithBall(*teamWithBall);
        if (*teamWithBall == teamID)
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
    // hide ball if far outside field
    if (abs(ball.position.x) < (0.5 * cEnvironmentField::getInstance().getWidth() + 2.0) &&
        abs(ball.position.y) < (0.5 * cEnvironmentField::getInstance().getLength() + 2.0))
    {
        balls.push_back(ball);
    }

    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    vecAdapter.adapt(&ballResult::position, balls); // adapt ball positions
    vecAdapter.adapt(&ballResult::velocity, balls); // adapt ball velocity
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
        obstacle.position = convert_xy(robot.getPosition());
        obstacle.velocity = convert_xy(robot.getVelocity());

        obstacles.push_back(obstacle);
    }
    addNonRobotObstaclesTo(gameData.nonRobotObstacles, obstacles);
    vecAdapter.adapt(&obstacleResult::position, obstacles); // adapt obstacle positions
    vecAdapter.adapt(&obstacleResult::velocity, obstacles); // adapt obstacle velocity
    r = rtdbConnection->put(OBSTACLES, &obstacles);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing OBSTACLES to RtDB");
    }

    T_ROBOT_STATE robot;
    robot.status = robotStatusEnum::INPLAY;
    robot.timestamp = ftime::now();
    robot.robotId = getRobotNumber(robotID);
    robot.teamId = "A"; // according to worldModel, ok to leave on A even for simulated team B thanks to database separation

    // robot FCS coordinates are already converted to their own FCS coords, no adapter needed
    robot.position = convert_xyphi(gameData.team.at(teamID).at(robotID).getPositionFCS());
    robot.velocity = convert_xyphi(gameData.team.at(teamID).at(robotID).getVelocityFCS());

    robot.hasBall = gameData.team.at(teamID).at(robotID).canKickBall(gameData.ball.getPosition());

    if (robot.hasBall)
    {
        robot.ballAcquired = convert_xy(gameData.ball.getPickupLocation());
        vecAdapter.adapt(robot.ballAcquired);
    }

    tprintf("put ROBOT_STATE robotId=%d status=%s position=[%6.2f, %6.2f, %6.2f] velocity=[%6.2f, %6.2f, %6.2f] hasBall=%s ballAcquired=[%6.2f, %6.2f]",
            robot.robotId,
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
        b.position = convert_xyz(pos);
        b.velocity = convert_xyz(vel);
        scene.balls.push_back(b);
    }

    // there are never any extra obstacles at simworld initialization

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
        TRACE("get SIMULATION_SCENE result=%d", r);
        tprintf("got new SimulationScene, result=%d", r);
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
    INIT_TRACE_THREAD("monitorScene")
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

