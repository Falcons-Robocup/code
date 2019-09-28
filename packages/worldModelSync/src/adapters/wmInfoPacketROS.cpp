 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * wmInfoPacketROS.cpp
 *
 *  Created on: Oct 12, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/wmInfoPacketROS.hpp"

#include "int/configurators/configuratorWorldModelPacket.hpp"
#include "int/constructors/packetConstructorWorldModel.hpp"
#include "int/types/ballMeasurement.hpp"
#include "int/types/ballPossession.hpp"
#include "int/types/obstacleMeasurement.hpp"
#include "int/types/robotLocation.hpp"

#include "WorldModelNames.h"
#include "cDiagnostics.hpp"
#include <ros/ros.h>
#include "tracing.hpp"
#include <FalconsCommon.h>

wmInfoPacketROS::wmInfoPacketROS()
{
	TRACE("construct");
}

wmInfoPacketROS::~wmInfoPacketROS()
/*
 * Chuck Norris is the reason Waldo is hiding
 */
{
	TRACE(">");

	TRACE("<");
}

void wmInfoPacketROS::initializeROS()
{
	TRACE("initializeROS");
	_hROS.reset(new ros::NodeHandle());
	_subWmSyncInfo = _hROS->subscribe(WorldModelInterface::t_wMSyncInfo, 1, &wmInfoPacketROS::wmSyncInfo_cb, this);
	TRACE("subscribed to topic %s", WorldModelInterface::t_wMSyncInfo.c_str());
}

void wmInfoPacketROS::wmSyncInfo_cb(const worldModel::t_wMSyncInfo::ConstPtr& msg)
{
	try
	{
		packetConstructorWorldModel packetConstructor;

		/*
		 * Fill ball measurements
		 */
		std::vector<ballMeasurement> balls;
		for(size_t i = 0; i < msg->nrBallMeasurements; i++)
		{
			ballMeasurement ball;

			ball.azimuth = msg->ballAzimuth.at(i);
			ball.elevation = msg->ballElevation.at(i);
			ball.radius = msg->ballRadius.at(i);
			ball.cameraX = msg->ballCameraX.at(i);
			ball.cameraY = msg->ballCameraY.at(i);
			ball.cameraZ = msg->ballCameraZ.at(i);
			ball.cameraPhi = msg->ballCameraPhi.at(i);
			ball.cameraType = msg->ballCameraType.at(i).camera_type;
			ball.confidence = msg->ballConfidence.at(i);
			ball.objectID = msg->ballObjectID.at(i);
			ball.timestamp = msg->ballTimestamp.at(i);
			ball.isValid = true;

			balls.push_back(ball);
		}
		packetConstructor.setBallMeasurements(balls);

		/*
		 * Fill obstacle measurements
		 */
		std::vector<obstacleMeasurement> obstacles;
		for(size_t i = 0; i < msg->nrObstacleMeasurements; i++)
		{
			obstacleMeasurement obstacle;

			obstacle.azimuth = msg->obstacleAzimuth.at(i);
			obstacle.elevation = msg->obstacleElevation.at(i);
			obstacle.radius = msg->obstacleRadius.at(i);
			obstacle.cameraX = msg->obstacleCameraX.at(i);
			obstacle.cameraY = msg->obstacleCameraY.at(i);
			obstacle.cameraZ = msg->obstacleCameraZ.at(i);
			obstacle.cameraPhi = msg->obstacleCameraPhi.at(i);
			obstacle.cameraType = msg->obstacleCameraType.at(i).camera_type;
			obstacle.confidence = msg->obstacleConfidence.at(i);
			obstacle.objectID = msg->obstacleObjectID.at(i);
			obstacle.timestamp = msg->obstacleTimestamp.at(i);
			obstacle.isValid = true;

			obstacles.push_back(obstacle);
		}
		packetConstructor.setObstacleMeasurements(obstacles);

		/*
		 * Setting ball possession
		 */
		packetConstructor.setBallPossession(
				((msg->possession.robotID == getRobotNumber()) &&
				(msg->possession.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER)));

		/*
		 * Setting group ID
		 */
		packetConstructor.setGroupID(configuratorWorldModelPacket::getInstance().getGroupID());

		/*
		 * Setting robot number
		 */
		packetConstructor.setRobotID(getRobotNumber());


		/*
		 * Setting robot location
		 */
		robotLocationStructure robotLocation;
		robotLocation.x = msg->locationX;
		robotLocation.y = msg->locationY;
		robotLocation.theta = msg->locationTheta;
		robotLocation.vx = msg->locationVx;
		robotLocation.vy = msg->locationVy;
		robotLocation.vtheta = msg->locationVtheta;
		robotLocation.confidence = msg->locationConfidence;
		robotLocation.timestamp = msg->locationTimestamp;
		packetConstructor.setRobotLocation(robotLocation);

        /* 
         * Packet counter for packetLoss diagnostics
         */
        static uint8_t counter = 0; // only 8 bits, rotates around
		packetConstructor.setPacketCounter(counter++);
		if (counter == 100) counter = 0; // TODO _PACKET_ARR_SIZE from cPacketLossAdministrator
         
		/*
		 * Notify that new world model information is available
		 */
		if(msg->ownRobotIsActive)
		{
		    // debug testing to give each robot a fixed packet loss:
		    // if (counter % getRobotNumber() == 0) _function(packetConstructor.getByteArray());

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
