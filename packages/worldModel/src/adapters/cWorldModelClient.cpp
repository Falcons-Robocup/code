// Copyright 2018-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldModelClient.cpp
 *
 *  Created on: Aug 18, 2018
 *      Author: Jan Feitsma
 */

#include "ext/cWorldModelClient.hpp"

#include "tracing.hpp"

cWorldModelClient::cWorldModelClient()
{
    setRobotId(getRobotNumber());
}

cWorldModelClient::~cWorldModelClient()
{
}

void cWorldModelClient::setRobotId(const int myRobotId)
{
    _myRobotId = myRobotId;
    auto teamChar = getTeamChar();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
}

void cWorldModelClient::update()
{
    TRACE_FUNCTION("");

    int r = 0;
    // clear
    _teamState.clear();
    _balls.clear();
    _obstacles.clear();
    // get robot- and team state
    for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
    {
        TRACE_SCOPE("AGENT", std::to_string(agentId).c_str());
        T_ROBOT_STATE item;
        int ageMs = 0;
        r = _rtdb->get(ROBOT_STATE, &item, ageMs, agentId);
        if (r == RTDB2_SUCCESS)
        {
            TRACE_SCOPE("ROBOT_STATE", std::to_string(agentId).c_str());
            // store
            if (agentId == _myRobotId)
            {
                TRACE_SCOPE("SELF", std::to_string(agentId).c_str());
                _robotState = item;
            }
            else if (ageMs < (ACTIVE_TIMEOUT*1000.0) ) // teammembers timeout: allow for wifi to stall for a few seconds
            {
                TRACE_SCOPE("TEAMMEMBER", std::to_string(agentId).c_str());
                _teamState[agentId] = item;
            }
        }
    }
    // get balls and obstacles
    T_BALLS balls;
    r = _rtdb->get(BALLS, &balls);
    if (r == RTDB2_SUCCESS)
    {
        _balls = balls;
    }
    T_OBSTACLES obstacles;
    r = _rtdb->get(OBSTACLES, &obstacles);
    if (r == RTDB2_SUCCESS)
    {
        _obstacles = obstacles;
    }

    // update configuration
    _configAdapter.get(_config);
    _myTeamId = _config.teamId;
}

void cWorldModelClient::update(const int myRobotId)
{
    // Overrule my robot ID
    int myOriginalRobotId = _myRobotId;
    setRobotId(myRobotId);

    // GET data from RTDB, store for all getters
    update();

    // Restore my robot ID
    setRobotId(myOriginalRobotId);
}

bool cWorldModelClient::isActive() const
{
    return _robotState.status == robotStatusEnum::INPLAY;
}

Position2D cWorldModelClient::getPosition() const
{
    return Position2D(_robotState.position.x, _robotState.position.y, _robotState.position.Rz);
}

Velocity2D cWorldModelClient::getVelocity() const
{
    return Velocity2D(_robotState.velocity.x, _robotState.velocity.y, _robotState.velocity.Rz);
}

bool cWorldModelClient::noBall() const
{
    return _balls.size() == 0;
}

bool cWorldModelClient::hasBall() const
{
    return _robotState.hasBall;
}

bool cWorldModelClient::teamHasBall() const
{
    // check ball & self
    if (noBall())
    {
        return false;
    }
    if (hasBall())
    {
        return true;
    }
    // check team members (excluding self)
    std::vector<T_ROBOT_STATE> teamMembers = getTeamMembersExcludingSelf();
    for (auto it = teamMembers.begin(); it != teamMembers.end(); ++it)
    {
        if (it->hasBall)
        {
            return true;
        }
    }
    return false;
}

bool cWorldModelClient::opponentHasBall() const
{
    if (noBall())
    {
        return false;
    }
    return _balls.at(0).owner.type == ballPossessionTypeEnum::OPPONENT;
}

Vector3D cWorldModelClient::ballPosition() const
{
    Vector3D result;
    if (_balls.size() > 0)
    {
        result = Vector3D(_balls[0].position.x, _balls[0].position.y, _balls[0].position.z);
    }
    return result;
}

Vector3D cWorldModelClient::ballVelocity() const
{
    Vector3D result;
    if (_balls.size() > 0)
    {
        result = Vector3D(_balls[0].velocity.x, _balls[0].velocity.y, _balls[0].velocity.z);
    }
    return result;
}

T_BALLS cWorldModelClient::getBalls() const
{
    return _balls;
}

T_OBSTACLES cWorldModelClient::getObstacles() const
{
    return _obstacles;
}

int cWorldModelClient::numObstacles() const
{
    return (int)_obstacles.size();
}

float cWorldModelClient::closestObstacleDistance() const
{
    float result2 = 9;
    auto currentPos = getPosition();
    for (auto it = _obstacles.begin(); it != _obstacles.end(); ++it)
    {
        float dx = (it->position.x - currentPos.x);
        float dy = (it->position.y - currentPos.y);
        float dist2 = dx * dx + dy * dy;
        if (dist2 < result2)
        {
            result2 = dist2;
        }
    }
    return sqrt(result2);
}

bool cWorldModelClient::getRobotState(T_ROBOT_STATE &robot, const int robotId)
{
    return getRobotState(robot, robotId, _myRobotId);
}

bool cWorldModelClient::getRobotState(T_ROBOT_STATE &robot, const int robotId, const int myRobotId)
{
    if (robotId == myRobotId)
    {
        robot = _robotState;
        return true;
    }
    if (!_teamState.count(robotId))
    {
        return false;
    }
    robot = _teamState[robotId];
    if (robot.teamId != _myTeamId)
    {
        return false;
    }
    return true;
}

std::vector<T_ROBOT_STATE> cWorldModelClient::getTeamMembersExcludingSelf() const
{
    TRACE_FUNCTION("");
    std::vector<T_ROBOT_STATE> result;

    std::map<int, T_ROBOT_STATE>::const_iterator it;
    for (it = _teamState.begin(); it != _teamState.end(); ++it)
    {
        if (it->first != _myRobotId)
        {
            if (it->second.teamId == _myTeamId)
            {
                TRACE("Found teammember %d", it->second.robotId);
                result.push_back(it->second);
            }
        }
    }

    return result;
}

int cWorldModelClient::getLowestActiveRobotID() const
{
    // init with own robot id
    int lowestActiveRobotID = _myRobotId;

    // iterate over teammates to find lower id
    for (const auto& teammate: _teamState)
    {
        if (teammate.first < lowestActiveRobotID)
        {
            lowestActiveRobotID = teammate.first;
        }
    }

    return lowestActiveRobotID;
}
