 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/RTDB/RTDBOutputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

RTDBOutputAdapter::RTDBOutputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    _ballPossessionType = ballPossessionTypeEnum::UNKNOWN;
    _ballPossessionRobot = 0;
}

RTDBOutputAdapter::~RTDBOutputAdapter()
/*
 * Chuck Norris tells Simon what to do
 */
{
}

void RTDBOutputAdapter::setBallAdministrator(ballAdministrator *ballAdmin)
{
    _ballAdmin = ballAdmin;
}

void RTDBOutputAdapter::setObstacleAdministrator(obstacleAdministrator *obstacleAdmin)
{
    _obstacleAdmin = obstacleAdmin;
}

void RTDBOutputAdapter::setRobotAdministrator(robotAdministrator *robotAdmin)
{
    _robotAdmin = robotAdmin;
}

void RTDBOutputAdapter::updateConfig(T_CONFIG_WORLDMODELSYNC const &config)
{
    _config = config;
}

void RTDBOutputAdapter::setRobotState()
{
    // do not write for coach (agent 0)
    if (_myRobotId == 0)
    {
        return;
    }
    
    TRACE_FUNCTION("");
    T_ROBOT_STATE state;

    if (_robotAdmin->getBallPossession())
    {
        // fill in ballPossession elements in _robotState
        _robotAdmin->getBallClaimedPosition(state.ballAcquired.x, state.ballAcquired.y);
        state.hasBall = true;
    }
    else
    {
        state.hasBall = false;
        state.ballAcquired.x = 0.0;
        state.ballAcquired.y = 0.0;
    }

    robotClass_t robotLocation = _robotAdmin->getLocalRobotPosition();

    state.timestamp = robotLocation.getTimestamp();
    state.position.x = robotLocation.getX();
    state.position.y = robotLocation.getY();
    state.position.Rz = robotLocation.getTheta();
    state.velocity.x = robotLocation.getVX();
    state.velocity.y = robotLocation.getVY();
    state.velocity.Rz = robotLocation.getVTheta();
    
    // team and robot id
    state.robotId = _myRobotId;
    state.teamId = _config.teamId;

    // init on outofplay, set to inplay if active
    state.status = T_INPLAY_FEEDBACK::OUTOFPLAY;

    std::vector<uint8_t> activeMembers = _robotAdmin->getActiveMembers();
    if(find(activeMembers.begin(), activeMembers.end(), getRobotNumber()) != activeMembers.end())
    {
        state.status = T_INPLAY_FEEDBACK::INPLAY;
    }

    _rtdb->put(ROBOT_STATE, &state);
}

void RTDBOutputAdapter::setBallPossession(ballPossessionTypeEnum bpType, int bpRobot)
{
    _ballPossessionType = bpType;
    _ballPossessionRobot = bpRobot;
}

void RTDBOutputAdapter::setBalls(std::vector<ballClass_t> const &balls)
{
    TRACE_FUNCTION("");
    /*
    * Only select the ball with the higher confidence
    * This is the first ball in the vector
    * Furthermore, give own balls with a radius of x meters prio over global balls
    * That calculation is done during ballDiscriminator::performCalculation
    */
    ballClass_t ball;
    TRACE("#balls=%d", (int)balls.size());
    if (balls.size())
    {
        ball = balls.at(0);
    }

    /*
    * Add own ball if ball is valid
    * If not, see if we have ball possession
    * If not, see if teammember has ball possession
    * Adjust ball location accordingly
    */
    TRACE("valid=%d", ball.getIsValid());

    T_BALLS ballResults;

    if (ball.getIsValid())
    {
        ballResult b;
        b.position.x = ball.getX();
        b.position.y = ball.getY();
        b.position.z = ball.getZ();
        b.velocity.x = ball.getVX();
        b.velocity.y = ball.getVY();
        b.velocity.z = ball.getVZ();
        b.confidence = ball.getConfidence();

        // Use ballpossession to update the ball's owner
        b.owner.type = _ballPossessionType;
        b.owner.robotId = _ballPossessionRobot;

        ballResults.push_back(b);
    }
    // TODO support multiple balls (dev/test/demo use case) - modify interface or call multiple times?

    _rtdb->put(BALLS, &ballResults);
}

void RTDBOutputAdapter::setObstacles()
{
    TRACE_FUNCTION("");
    std::vector<obstacleClass_t> obstacles;
    _obstacleAdmin->getObstacles(obstacles);

    T_OBSTACLES obstacleResults;
    for (auto it = obstacles.begin(); it != obstacles.end(); it++)
    {
        obstacleResult obst;
        obst.position.x = it->getX();
        obst.position.y = it->getY();
        obst.velocity.x = it->getVX();
        obst.velocity.y = it->getVY();
        obst.confidence = it->getConfidence();
        obst.id = it->getId();
        obstacleResults.push_back(obst);
    }

    _rtdb->put(OBSTACLES, &obstacleResults);
}

