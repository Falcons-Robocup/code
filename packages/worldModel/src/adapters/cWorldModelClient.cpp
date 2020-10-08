 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelClient.cpp
 *
 *  Created on: Aug 18, 2018
 *      Author: Jan Feitsma
 */

#include "ext/cWorldModelClient.hpp"

cWorldModelClient::cWorldModelClient()
{
}

cWorldModelClient::~cWorldModelClient()
{
}

void cWorldModelClient::update()
{
    int r = 0;
    // clear
    _teamState.clear();
    _balls.clear();
    _obstacles.clear();
    // get robot- and team state
    for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
    {
        RtDB2Item item;
        r = _rtdb.at(*it)->getItem(ROBOT_STATE, item, *it);
        if (r == RTDB2_SUCCESS)
        {
            // store
            if (*it == _myRobotId)
            {
                _robotState = item.value<T_ROBOT_STATE>();
            }
            else if (item.age() < ACTIVE_TIMEOUT) // teammembers timeout: allow for wifi to stall for a few seconds
            {
                _teamState[*it] = item.value<T_ROBOT_STATE>();
            }
        }
    }
    // get balls and obstacles
    if (_rtdb.count(_myRobotId))
    {
        T_BALLS balls;
        r = _rtdb.at(_myRobotId)->get(BALLS, &balls);
        if (r == RTDB2_SUCCESS)
        {
            _balls = balls;
        }
        T_OBSTACLES obstacles;
        r = _rtdb.at(_myRobotId)->get(OBSTACLES, &obstacles);
        if (r == RTDB2_SUCCESS)
        {
            _obstacles = obstacles;
        }
    }
    // update configuration
    _configAdapter.get(_config);
    _myTeamId = _config.teamId;
}

void cWorldModelClient::update(const int myRobotId)
{
    // Overrule my robot ID
    _myRobotId = myRobotId;

    // GET data from RTDB, store for all getters
    update();

    // Restore my robot ID
    _myRobotId = getRobotNumber();
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
    std::vector<T_ROBOT_STATE> result;

    std::map<int, T_ROBOT_STATE>::const_iterator it;
    for (it = _teamState.begin(); it != _teamState.end(); ++it)
    {
        if (it->first != _myRobotId)
        {
            if (it->second.teamId == _myTeamId)
            {
                result.push_back(it->second);
            }
        }
    }

    return result;
}
