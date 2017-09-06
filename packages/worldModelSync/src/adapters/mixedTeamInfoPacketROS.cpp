 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mixedTeamInfoPacketROS.cpp
 *
 *  Created on: Jan 22, 2017
 *      Author: Tim Kouters
 */

#include "int/adapters/mixedTeamInfoPacketROS.hpp"

#include "int/configurators/configuratorMixedTeamPacket.hpp"
#include "int/constructors/packetConstructorMixedTeam.hpp"
#include "int/types/ballCandidate.hpp"
#include "int/types/obstacleCandidate.hpp"
#include "int/types/robotLocationMixedTeam.hpp"
#include "int/types/mixedTeamConversionConstants.hpp"

#include "WorldModelNames.h"
#include "cDiagnosticsEvents.hpp"
#include <ros/ros.h>
#include <tracer.hpp>
#include <FalconsCommon.h>

mixedTeamInfoPacketROS::mixedTeamInfoPacketROS()
{
	TRACE("construct");
}

mixedTeamInfoPacketROS::~mixedTeamInfoPacketROS()
{
	TRACE(">");

	TRACE("<");
}

void mixedTeamInfoPacketROS::initializeROS()
{
	TRACE("initializeROS");
	_hROS.reset(new ros::NodeHandle());
	_subWmInfo = _hROS->subscribe(WorldModelInterface::t_wmInfo, 1, &mixedTeamInfoPacketROS::wmInfo_cb, this);
	TRACE("subscribed to topic %s", WorldModelInterface::t_wmInfo.c_str());
}

void mixedTeamInfoPacketROS::wmInfo_cb(const worldModel::t_wmInfo::ConstPtr& msg)
{
	try
	{
		packetConstructorMixedTeam packetConstructor;

		/*
		 * Fill ball measurements
		 */
		std::vector<ballCandidateStructure> balls;
		ballCandidateStructure ball;

		ball.x = msg->ballX* M2MM;
		ball.y = msg->ballY* M2MM;
		ball.z = msg->ballZ* M2MM;
		ball.vx = msg->ballVX* M2MM;
		ball.vy = msg->ballVY* M2MM;
		ball.vz = msg->ballVZ * M2MM;
		ball.confidence = msg->ballConfidence * CONF_FACTOR_MIXED;

		balls.push_back(ball);
		packetConstructor.setBallCandidates(balls);

		/*
		 * Fill obstacle measurements
		 */
		std::vector<obstacleCandidateStructure> obstacles;
		for(size_t i = 0; i < msg->nrObstacleMeasurements; i++)
		{
			obstacleCandidateStructure obstacle;

			obstacle.x = msg->obstacleX.at(i) * M2MM;
			obstacle.y = msg->obstacleY.at(i) * M2MM;
			obstacle.vx = msg->obstacleVX.at(i) * M2MM;
			obstacle.vy = msg->obstacleVY.at(i) * M2MM;
			obstacle.confidence = msg->obstacleConfidence.at(i) * CONF_FACTOR_MIXED;

			obstacles.push_back(obstacle);
		}
		packetConstructor.setObstacleCandidates(obstacles);


		/*
		 * Setting group ID
		 */
		// TODO: TKOV

		/*
		 * Color of robot
		 */
		// TODO: TKOV

		/*
		 * Setting robot number
		 */
		packetConstructor.setOriginalTeamId(TEAM_ID);


		/*
		 * Setting robot location
		 */
		robotLocationMixedTeamStructure robotLocation;
		robotLocation.x = msg->locationX * M2MM;
		robotLocation.y = msg->locationY * M2MM;
		robotLocation.theta = msg->locationTheta * M2MM;
		robotLocation.vx = msg->locationVx * M2MM;
		robotLocation.vy = msg->locationVy * M2MM;
		robotLocation.vtheta = msg->locationVtheta * M2MM;
		robotLocation.confidence = msg->ballConfidence * CONF_FACTOR_MIXED;
		packetConstructor.setRobotLocation(robotLocation);


		/*
		 * Notify that new world model information is available
		 */
		if(msg->ownRobotIsActive)
		{
			_function(packetConstructor.getByteArray());
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

}
