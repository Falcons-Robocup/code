 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/cRTDBInputAdapter.hpp"

#include "int/cWorldModelInterface.hpp"
#include "int/worldModelInfo.hpp"
#include "cDiagnostics.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

cRTDBInputAdapter::cRTDBInputAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
    TRACE("<");
}

void cRTDBInputAdapter::waitForRobotState(void (*iterateTeamplayFuncPtr) (void))
{
    while (true)
    {
        _rtdb->waitForPut(TP_HEARTBEAT);

        iterateTeamplayFuncPtr();
    }
}

void cRTDBInputAdapter::getWorldModelData()
{
    getWorldModelData(_myRobotId);
}

void cRTDBInputAdapter::getWorldModelData(const int robotID)
{
    TRACE_FUNCTION("");

    // poke worldModel to update
    _wmClient.update(robotID);

    // convert data to teamplay::worldModelInfo
    // TODO: replace all this type duplications and conversions with direct gets on wmClient
    teamplay::worldModelInfo wmInfo;

    // convert own robot position and velocity
    // TODO use robot.status, it is not part of teamplay::worldModelInfoOwnRobot - see ticket timmel:#614
    wmInfo.ownRobot.number = robotID;
    wmInfo.ownRobot.position = _wmClient.getPosition();
    wmInfo.ownRobot.velocity = _wmClient.getVelocity();

    // convert ball possession and teammember locations
    wmInfo.ballPossession.possessionType = ballPossessionEnum::FIELD;
    wmInfo.ballPossession.robotID = 0;
    for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
    {
        robotState robot;
        if (_wmClient.getRobotState(robot, agentId, robotID))
        {
            if (robot.hasBall)
            {
                wmInfo.ballPossession.possessionType = ballPossessionEnum::TEAMMEMBER;
                wmInfo.ballPossession.robotID = agentId;
                wmInfo.ballPossession.ballClaimedLocation.x = robot.ballAcquired.x;
                wmInfo.ballPossession.ballClaimedLocation.y = robot.ballAcquired.y;
            }

            if (agentId != robotID && robot.status == robotStatusEnum::INPLAY)
            {
                robotLocation loc;
                loc.position = geometry::Pose2D(robot.position.x, robot.position.y, robot.position.Rz);
                loc.velocity = geometry::Velocity2D(robot.velocity.x, robot.velocity.y, robot.velocity.Rz);
                wmInfo.activeTeammembers[agentId] = loc;
            }
        }
    }

    // convert balls
    auto balls = _wmClient.getBalls();
    if (balls.size() == 0)
    {
        wmInfo.ball = boost::none;
    }
    else
    {
        // expect best ball to be the first in this array
        // TODO: do something with other balls (dev/test/demo use cases)
        ballLocation ball;
        auto b = balls[0];
        ball.position.x = b.position.x;
        ball.position.y = b.position.y;
        ball.position.z = b.position.z;
        ball.velocity.x = b.velocity.x;
        ball.velocity.y = b.velocity.y;
        ball.velocity.z = b.velocity.z;
        ball.confidence = b.confidence;
        wmInfo.ball = ball;
    }

    // convert obstacles
    auto obstacles = _wmClient.getObstacles();
    for (int it = 0; it < (int)obstacles.size(); ++it)
    {
        auto obst = obstacles.at(it);
        robotLocation loc;
        loc.position = geometry::Pose2D(obst.position.x, obst.position.y, 0.0);
        loc.velocity = geometry::Velocity2D(obst.velocity.x, obst.velocity.y, 0.0);
        // why must we store it as a map? (a vector seems more natural)
        wmInfo.obstacles[it] = loc;
    }

    // store
    cWorldModelInterface::getInstance().store(wmInfo);
}

std::string cRTDBInputAdapter::getRole(const int robotID)
{
    T_ROBOT_ROLE robotRole;

    auto r = _rtdb->get(ROBOT_ROLE, &robotRole, robotID);
    if (r != RTDB2_SUCCESS)
    {
        tprintf("rtdb/tp/getRole failure");
    }

    return robotRole;
}

T_INTENTION cRTDBInputAdapter::getIntention(const int robotID)
{
    T_INTENTION intention;

    auto r = _rtdb->get(INTENTION, &intention, robotID);
    if (r != RTDB2_SUCCESS)
    {
        tprintf("rtdb/tp/getIntention failure");
    }

    return intention;
}
