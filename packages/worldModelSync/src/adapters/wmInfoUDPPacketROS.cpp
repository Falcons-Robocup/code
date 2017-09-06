 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * wmInfoUDPPacketROS.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/wmInfoUDPPacketROS.hpp"

#include "int/types/packetStructureWorldModel.hpp"
#include "int/constructors/packetConstructorWorldModel.hpp"
#include "int/configurators/configuratorWorldModelPacket.hpp"

#include <worldModel/set_remote_ball_location.h>
#include <worldModel/set_remote_ball_possession.h>
#include <worldModel/set_remote_obstacle_location.h>
#include <worldModel/set_member_location.h>
#include "WorldModelNames.h"

#include "cDiagnosticsEvents.hpp"
#include "tracer.hpp"
#include "FalconsCommon.h" // getRobotNumber
#include <ros/ros.h>

#define PACKETLOSS_THRESHOLD 10.0 // percentage
#define PACKETGAP_THRESHOLD 0.5 // seconds

wmInfoUDPPacketROS::wmInfoUDPPacketROS()
{
	_servicesAreValid = false;
	TRACE("construct");
}

wmInfoUDPPacketROS::~wmInfoUDPPacketROS()
{
	TRACE(">");

	TRACE("<");
}

void wmInfoUDPPacketROS::initializeROS()
{
	TRACE("initializeROS");
	try
	{
		_hROS.reset(new ros::NodeHandle());

        TRACE("waiting for services");
		ros::service::waitForService(WorldModelInterface::s_set_remote_ball_location);
		ros::service::waitForService(WorldModelInterface::s_set_remote_ball_possession);
		ros::service::waitForService(WorldModelInterface::s_set_remote_obstacle_location);
		ros::service::waitForService(WorldModelInterface::s_set_member_location);
        TRACE("all services online");

		_srvSetRemoteBallLocations = _hROS->serviceClient<worldModel::set_remote_ball_location>(WorldModelInterface::s_set_remote_ball_location);
		_srvSetRemoteBallPossession = _hROS->serviceClient<worldModel::set_remote_ball_possession>(WorldModelInterface::s_set_remote_ball_possession);
		_srvSetRemoteObstacleLocations = _hROS->serviceClient<worldModel::set_remote_obstacle_location>(WorldModelInterface::s_set_remote_obstacle_location);
		_srvSetRemoteRobotLocation = _hROS->serviceClient<worldModel::set_member_location>(WorldModelInterface::s_set_member_location);

		_servicesAreValid = true;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

static void checkPacketGap(packetStructureWorldModel const &wmStruct)
{
    // check if we suffer from poor wifi -- long gaps
    // TODO: more fine-grained: measure frequency, check if significantly lower than heartbeat?
    // TODO: when robot goes in play or out of play, this warning will trigger as false positive
    static std::map<int, double> lastTimeStamps; 
    // we use the feature that map values are auto-initialized to zero
    double gapSize = wmStruct.robotPosition.timestamp - lastTimeStamps[wmStruct.robotID];
    double gapSizeThreshold = PACKETGAP_THRESHOLD;
    if (gapSize > gapSizeThreshold)
    {
        if (fabs(gapSize) < 10) // filter false positives due to init
        {
            TRACE_WARNING_TIMEOUT(1.0, "gap of %.2fs in wmSync packet stream from r%d", gapSize, (int)(wmStruct.robotID));
        }
    }
    lastTimeStamps[wmStruct.robotID] = wmStruct.robotPosition.timestamp;
}

void wmInfoUDPPacketROS::notifyNewUDPPacket(Facilities::Network::cByteArray array)
{
	try
	{
		if(_servicesAreValid)
		{
			packetConstructorWorldModel packetConstructor;
			packetConstructor.setByteArray(array);
			packetStructureWorldModel wmStruct = packetConstructor.getWorldModelStructure();

			/*
			 * Only sync if part of group ID and not own robotID
			 */
			if((wmStruct.groupID == configuratorWorldModelPacket::getInstance().getGroupID()) &&
					(wmStruct.robotID != getRobotNumber()) &&
					(wmStruct.packetVersion = 2) )
			{
			    /*
			     * Check packet gaps (timed) and packet loss statistics
			     */
        	    checkPacketGap(wmStruct);
        	    _packetLossAdministrator.notifyPacketNumber(wmStruct.packetCounter, wmStruct.robotID);
        	    float packetLossPercentage = _packetLossAdministrator.getPacketLoss(wmStruct.robotID);
        	    //TRACE("packetLossPercentage=%.1f%%", packetLossPercentage);
                if (packetLossPercentage > PACKETLOSS_THRESHOLD) // filter false positives due to init
                {
                    TRACE_WARNING_TIMEOUT(5.0, "high packet loss (%d%%) in wmSync packet stream from r%d", (int)packetLossPercentage, (int)(wmStruct.robotID));
                }
        	    
				/*
				 * Fill and send robot location
				 */
				worldModel::set_member_location memberLocation;

				memberLocation.request.robotID = wmStruct.robotID;
				memberLocation.request.robotPos.positionX = wmStruct.robotPosition.x;
				memberLocation.request.robotPos.positionY = wmStruct.robotPosition.y;
				memberLocation.request.robotPos.positionTheta = wmStruct.robotPosition.theta;
				memberLocation.request.robotPos.velocityX = wmStruct.robotPosition.vx;
				memberLocation.request.robotPos.velocityY = wmStruct.robotPosition.vy;
				memberLocation.request.robotPos.velocityTheta = wmStruct.robotPosition.vtheta;
				memberLocation.request.robotPos.confidence = wmStruct.robotPosition.confidence;
				memberLocation.request.robotPos.timestamp = wmStruct.robotPosition.timestamp;

				if(!_srvSetRemoteRobotLocation.call(memberLocation))
				{
					TRACE_ERROR("Failed to set remote robot location for robot %d", wmStruct.robotID);
					initializeROS();
				}


				/*
				 * Fill and send ball locations
				 */
				worldModel::set_remote_ball_location remoteBalls;

				for(size_t i = 0; ((i < NR_BALL_MEASUREMENTS) && (wmStruct.balls[i].isValid)); i++)
				{
					remoteBalls.request.confidence.push_back(wmStruct.balls[i].confidence);

					remoteBalls.request.cameraX.push_back(wmStruct.balls[i].cameraX);
					remoteBalls.request.cameraY.push_back(wmStruct.balls[i].cameraY);
					remoteBalls.request.cameraZ.push_back(wmStruct.balls[i].cameraZ);
					remoteBalls.request.cameraPhi.push_back(wmStruct.balls[i].cameraPhi);

					rosMsgs::s_camera_type cameraType;
					cameraType.camera_type = wmStruct.balls[i].cameraType;
					remoteBalls.request.cameraType.push_back(cameraType);

					remoteBalls.request.azimuth.push_back(wmStruct.balls[i].azimuth);
					remoteBalls.request.elevation.push_back(wmStruct.balls[i].elevation);
					remoteBalls.request.radius.push_back(wmStruct.balls[i].radius);

					remoteBalls.request.objectID.push_back(wmStruct.balls[i].objectID);
					remoteBalls.request.timestamp.push_back(wmStruct.balls[i].timestamp);
					remoteBalls.request.robotID.push_back(wmStruct.robotID);
				}

				if(!_srvSetRemoteBallLocations.call(remoteBalls))
				{
					TRACE_ERROR("Failed to set remote ball location for robot %d", wmStruct.robotID);
					initializeROS();
				}

				/*
				 * Fill and send ball possession
				 */
				worldModel::set_remote_ball_possession ballPossession;

				ballPossession.request.possession.robotID = wmStruct.robotID;
				if(wmStruct.ballPossession.hasBallPossession)
				{
					ballPossession.request.possession.type = rosMsgs::BallPossession::TYPE_TEAMMEMBER;
				}
				else
				{
					ballPossession.request.possession.type = rosMsgs::BallPossession::TYPE_FIELD;
				}

				if(!_srvSetRemoteBallPossession.call(ballPossession))
				{
					TRACE_ERROR("Failed to set remote ball possession for robot %d", wmStruct.robotID);
					initializeROS();
				}

				/*
				 * Fill and send obstacle locations
				 */
				worldModel::set_remote_obstacle_location remoteObstacles;

				for(size_t i = 0; ((i < NR_OBSTACLE_MEASUREMENTS) && (wmStruct.obstacles[i].isValid)); i++)
				{
					remoteObstacles.request.confidence.push_back(wmStruct.obstacles[i].confidence);

					remoteObstacles.request.cameraX.push_back(wmStruct.obstacles[i].cameraX);
					remoteObstacles.request.cameraY.push_back(wmStruct.obstacles[i].cameraY);
					remoteObstacles.request.cameraZ.push_back(wmStruct.obstacles[i].cameraZ);
					remoteObstacles.request.cameraPhi.push_back(wmStruct.obstacles[i].cameraPhi);

					rosMsgs::s_camera_type cameraType;
					cameraType.camera_type = wmStruct.obstacles[i].cameraType;
					remoteObstacles.request.cameraType.push_back(cameraType);

					remoteObstacles.request.azimuth.push_back(wmStruct.obstacles[i].azimuth);
					remoteObstacles.request.elevation.push_back(wmStruct.obstacles[i].elevation);
					remoteObstacles.request.radius.push_back(wmStruct.obstacles[i].radius);

					remoteObstacles.request.objectID.push_back(wmStruct.obstacles[i].objectID);
					remoteObstacles.request.timestamp.push_back(wmStruct.obstacles[i].timestamp);
					remoteObstacles.request.robotID.push_back(wmStruct.robotID);
				}

				if(!_srvSetRemoteObstacleLocations.call(remoteObstacles))
				{
					TRACE_ERROR("Failed to set remote obstacles for robot %d", wmStruct.robotID);
					initializeROS();
				}
			}
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
