 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterLogging.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Jan Feitsma
 */

#include <stdexcept>

#include <FalconsCommon.h>
#include "int/cRosAdapterLogging.hpp"
#include "int/logger/cPacketRefboxLogger.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

using namespace std;

static const int nrOfDevices = 7;  // 1 coach + 6 robots

cRosAdapterLogging::cRosAdapterLogging()
{
	_wmTeam = _nh.subscribe<rosMsgs::t_worldmodel_team>("g_worldmodel_team", 1, &cRosAdapterLogging::cb_worldModelUpdated, this);

    for (int i = 0; i < nrOfDevices; i++)
    {
        boost::format teamplayTopicFormatter("/teamA/robot%1%/g_diag_teamplay");
        teamplayTopicFormatter % std::to_string(i);
        _teamplaySubscriptionPerRobot.push_back(_nh.subscribe<rosMsgs::t_diag_teamplay>(teamplayTopicFormatter.str(), 1, boost::bind(&cRosAdapterLogging::cb_teamplayUpdated, this, _1, i)));
        _lastTeamplayMessagePerRobot.push_back(rosMsgs::t_diag_teamplay());

        boost::format pathplanningTopicFormatter("/teamA/robot%1%/g_diag_pathpl");
        pathplanningTopicFormatter % std::to_string(i);
        _pathplanningSubscriptionPerRobot.push_back(_nh.subscribe<rosMsgs::t_diag_pathpl>(pathplanningTopicFormatter.str(), 1, boost::bind(&cRosAdapterLogging::cb_pathplanningUpdated, this, _1, i)));
        _lastPathplanningMessagePerRobot.push_back(rosMsgs::t_diag_pathpl());
    }

    TRACE("cRosAdapterLogging constructed");
}

void cRosAdapterLogging::cb_worldModelUpdated(const rosMsgs::t_worldmodel_team::ConstPtr& msg)
{
    _lastWmMsg = *msg;
}

void cRosAdapterLogging::cb_teamplayUpdated(const rosMsgs::t_diag_teamplay::ConstPtr& msg, int robotID)
{
    _lastTeamplayMessagePerRobot[robotID] = *msg;
}

void cRosAdapterLogging::cb_pathplanningUpdated(const rosMsgs::t_diag_pathpl::ConstPtr& msg, int robotID)
{
    _lastPathplanningMessagePerRobot[robotID] = *msg;
}

void cRosAdapterLogging::updateRobot(packetRefboxLogger::cPacketRefboxLogger &logPacket)
{
    for (size_t it = 0; it < nrOfDevices; ++it)
    {
        if (_lastWmMsg.active.size() > it && _lastWmMsg.active[it])
        {
            // worldModel position and velocity
            Position2D pose;
            Position2D velocity;
            uint8_t robotID = it;
            rosMsgs::t_posvel robot = _lastWmMsg.robots.at(it);
            pose.x = robot.x;
            pose.y = robot.y;
            pose.phi = robot.phi;
            logPacket.setRobotPose(robotID, pose);
            velocity.x = robot.vx;
            velocity.y = robot.vy;
            velocity.phi = robot.vphi;
            logPacket.setRobotVelocity(robotID, velocity);

            // worldModel ball possession
            bool have_ball = (_lastWmMsg.ballPossession.robotID == robotID);
            logPacket.setRobotBallPossession(robotID, have_ball);

            // pathplanning target
            Position2D targetpose;
            targetpose.x = _lastPathplanningMessagePerRobot[robotID].target.x;
            targetpose.y = _lastPathplanningMessagePerRobot[robotID].target.y;
            targetpose.phi = _lastPathplanningMessagePerRobot[robotID].target.phi;
            logPacket.setRobotTargetPose(robotID, targetpose);

            // teamplay intention: combination of role and behavior
            boost::format fmt("%1%: %2%");
            fmt % _lastTeamplayMessagePerRobot[robotID].role;
            fmt % _lastTeamplayMessagePerRobot[robotID].behavior;
            logPacket.setRobotIntention(robotID, fmt.str());

            // TODO battery level
        }
    }
}

void cRosAdapterLogging::updateBall(packetRefboxLogger::cPacketRefboxLogger &logPacket)
{
    // ball possession already done in updateRobot

    // only relay the first ball, if any
    if (_lastWmMsg.balls.size() > 0)
    {
        Vector3D position, velocity;
        position.x = _lastWmMsg.balls[0].x;
        position.y = _lastWmMsg.balls[0].y;
        position.z = _lastWmMsg.balls[0].z;
        velocity.x = _lastWmMsg.balls[0].vx;
        velocity.y = _lastWmMsg.balls[0].vy;
        velocity.z = _lastWmMsg.balls[0].vz;
        float confidence = 1.0;
        logPacket.addBall(position, velocity, confidence);
    }
}

void cRosAdapterLogging::updateObstacles(packetRefboxLogger::cPacketRefboxLogger &logPacket)
{
    size_t numObstacles = _lastWmMsg.obstacles.size();
    for (size_t it = 0; it < numObstacles; ++it)
    {
        Vector2D position, velocity;
        float confidence = 1.0; // unused
        position.x = _lastWmMsg.obstacles.at(it).x;
        position.y = _lastWmMsg.obstacles.at(it).y;
        velocity.x = _lastWmMsg.obstacles.at(it).vx;
        velocity.y = _lastWmMsg.obstacles.at(it).vy;
        logPacket.addObstacle(position, velocity, confidence);
    }
}

void cRosAdapterLogging::update(CTCPIP_Client *tcpip_client)
{
	TRACE("update");
	packetRefboxLogger::cPacketRefboxLogger logPacket;

	// fill the data
	logPacket.setType("worldstate");
	logPacket.setTeamIntention("undefined"); // TODO how to determine from teamplay?

	// update the robots
	updateRobot(logPacket);

	// ball position
	updateBall(logPacket);

	// obstacles
	updateObstacles(logPacket);

	// age
	logPacket.setAgeMilliseconds(0); // TODO how to determine age?

	// serialize
	std::string jsonMessage;
	logPacket.getSerialized(jsonMessage);
	// put on the line to refbox
	TRACE(jsonMessage.data());
	tcpip_client->Send((uint8_t*)jsonMessage.data(), jsonMessage.size());
}
