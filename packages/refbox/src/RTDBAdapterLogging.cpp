 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBAdapterLogging.cpp
 *
 *  Created on: Jun 14, 2019
 *      Author: Jan Feitsma
 */

#include <stdexcept>

#include <FalconsCommon.h>
#include "tracing.hpp"
#include "int/RTDBAdapterLogging.hpp"
#include "int/logger/cPacketRefboxLogger.hpp"

#include <boost/bind.hpp>

RTDBAdapterLogging::RTDBAdapterLogging()
{
}

void RTDBAdapterLogging::updateRobots(packetRefboxLogger::cPacketRefboxLogger &logPacket)
{
    for (int agentId = 1; agentId <= MAX_ROBOTS; ++agentId)
    {
        robotState robot;
        if (_wmClient.getRobotState(robot, agentId))
        {
            if (robot.status == robotStatusEnum::INPLAY)
            {
                // worldModel position and velocity
                pose p = robot.position;
                // correct robot orientation from Falcons FCS to general MSL coordinate system
                p.Rz = project_angle_0_2pi(p.Rz - 0.5*M_PI);
                logPacket.setRobotPose(agentId, p);
                logPacket.setRobotVelocity(agentId, robot.velocity);
                // worldModel ball possession
                logPacket.setRobotBallPossession(agentId, robot.hasBall);
                // TODO get data from pathPlanning
                //logPacket.setRobotTargetPose(agentId, targetpose);
                // TODO teamplay intention: combination of role and behavior
                //boost::format fmt("%1%: %2%");
                //fmt % _lastTeamplayMessagePerRobot[robotID].role;
                //fmt % _lastTeamplayMessagePerRobot[robotID].behavior;
                //logPacket.setRobotIntention(robotID, fmt.str());
                // TODO battery level
            }
        }
    }
}

void RTDBAdapterLogging::updateBall(packetRefboxLogger::cPacketRefboxLogger &logPacket)
{
    // ball possession already done in updateRobot
    if (!_wmClient.noBall())
    {
        ballResult ball = _wmClient.getBalls()[0]; // best ball is in the front
        float confidence = 1.0; // don't share
        logPacket.addBall(ball.position, ball.velocity, confidence);
    }
}

void RTDBAdapterLogging::updateObstacles(packetRefboxLogger::cPacketRefboxLogger &logPacket)
{
    T_OBSTACLES obstacles = _wmClient.getObstacles();
    size_t numObstacles = obstacles.size();
    for (size_t it = 0; it < numObstacles; ++it)
    {
        float confidence = 1.0; // unused
        logPacket.addObstacle(obstacles.at(it).position, obstacles.at(it).velocity, confidence);
    }
}

void RTDBAdapterLogging::update(CTCPIP_Client *tcpip_client)
{
    TRACE("update");
    packetRefboxLogger::cPacketRefboxLogger logPacket;

    // trigger worldModel client to fetch data
    _wmClient.update();

    // fill the data
    logPacket.setType("worldstate");
    logPacket.setTeamIntention("undefined"); // TODO

    // update the robots
    updateRobots(logPacket);

    // ball position
    updateBall(logPacket);

    // obstacles
    updateObstacles(logPacket);

    // age
    logPacket.setAgeMilliseconds(0); // TODO

    // serialize
    std::string jsonMessage;
    logPacket.getSerialized(jsonMessage);
    // put on the line to refbox
    //tprintf("json: %s", jsonMessage.c_str());
    TRACE(jsonMessage.c_str());
    if (tcpip_client != NULL)
    {
        tcpip_client->Send((uint8_t*)jsonMessage.data(), jsonMessage.size());
    }
}

