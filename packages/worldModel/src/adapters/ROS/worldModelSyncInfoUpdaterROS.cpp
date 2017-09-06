 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * worldModelSyncInfoUpdaterROS.cpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/ROS/worldModelSyncInfoUpdaterROS.hpp"

#include "ext/WorldModelNames.h"
#include "int/facilities/ROSConvert.hpp"

#include "cDiagnosticsEvents.hpp"

worldModelSyncInfoUpdaterROS::worldModelSyncInfoUpdaterROS()
{

}

worldModelSyncInfoUpdaterROS::~worldModelSyncInfoUpdaterROS()
/*
 * Chuck Norris knows Victoria's secret
 */
{

}

void worldModelSyncInfoUpdaterROS::InitializeROS()
{
	try
	{
		_hROS.reset(new ros::NodeHandle());
		_pWmInfo = _hROS->advertise<worldModel::t_wMSyncInfo>(WorldModelInterface::t_wMSyncInfo, 2, false);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void worldModelSyncInfoUpdaterROS::setBallLocations(std::vector<ballMeasurementType> balls)
{
	_msgWmSyncInfo.ballAzimuth.clear();
	_msgWmSyncInfo.ballElevation.clear();
	_msgWmSyncInfo.ballRadius.clear();
	_msgWmSyncInfo.ballCameraX.clear();
	_msgWmSyncInfo.ballCameraY.clear();
	_msgWmSyncInfo.ballCameraZ.clear();
	_msgWmSyncInfo.ballCameraPhi.clear();
	_msgWmSyncInfo.ballCameraType.clear();
	_msgWmSyncInfo.ballConfidence.clear();
	_msgWmSyncInfo.ballObjectID.clear();
	_msgWmSyncInfo.ballTimestamp.clear();

	_msgWmSyncInfo.nrBallMeasurements = balls.size();
	for(auto it = balls.begin(); it != balls.end(); it++)
	{
		_msgWmSyncInfo.ballAzimuth.push_back(it->getAzimuth());
		_msgWmSyncInfo.ballElevation.push_back(it->getElevation());
		_msgWmSyncInfo.ballRadius.push_back(it->getRadius());
		_msgWmSyncInfo.ballCameraX.push_back(it->getCameraX());
		_msgWmSyncInfo.ballCameraY.push_back(it->getCameraY());
		_msgWmSyncInfo.ballCameraZ.push_back(it->getCameraZ());
		_msgWmSyncInfo.ballCameraPhi.push_back(it->getCameraPhi());
		_msgWmSyncInfo.ballCameraType.push_back(convertCameraType(it->getCameraType()));
		_msgWmSyncInfo.ballConfidence.push_back(it->getConfidence());
		_msgWmSyncInfo.ballObjectID.push_back(it->getID().uniqueID);
		_msgWmSyncInfo.ballTimestamp.push_back(it->getTimestamp());
	}
}

void worldModelSyncInfoUpdaterROS::setBallPossession(ballPossessionClass_t possession)
{
	_msgWmSyncInfo.possession = convertBallPossession(possession);
}

void worldModelSyncInfoUpdaterROS::setObstacles(std::vector<obstacleMeasurementType> obstacles)
{
	_msgWmSyncInfo.obstacleConfidence.clear();
	_msgWmSyncInfo.obstacleTimestamp.clear();
	_msgWmSyncInfo.obstacleAzimuth.clear();
	_msgWmSyncInfo.obstacleElevation.clear();
	_msgWmSyncInfo.obstacleRadius.clear();
	_msgWmSyncInfo.obstacleCameraX.clear();
	_msgWmSyncInfo.obstacleCameraY.clear();
	_msgWmSyncInfo.obstacleCameraZ.clear();
	_msgWmSyncInfo.obstacleCameraPhi.clear();
	_msgWmSyncInfo.obstacleCameraType.clear();
	_msgWmSyncInfo.obstacleObjectID.clear();

	_msgWmSyncInfo.nrObstacleMeasurements = obstacles.size();
	for(auto it = obstacles.begin(); it != obstacles.end(); it++)
	{
		_msgWmSyncInfo.obstacleConfidence.push_back(it->getConfidence());
		_msgWmSyncInfo.obstacleTimestamp.push_back(it->getTimestamp());
		_msgWmSyncInfo.obstacleAzimuth.push_back(it->getAzimuth());
		_msgWmSyncInfo.obstacleElevation.push_back(it->getElevation());
		_msgWmSyncInfo.obstacleRadius.push_back(it->getRadius());
		_msgWmSyncInfo.obstacleCameraX.push_back(it->getCameraX());
		_msgWmSyncInfo.obstacleCameraY.push_back(it->getCameraY());
		_msgWmSyncInfo.obstacleCameraZ.push_back(it->getCameraZ());
		_msgWmSyncInfo.obstacleCameraPhi.push_back(it->getCameraPhi());
		_msgWmSyncInfo.obstacleCameraType.push_back(convertCameraType(it->getCameraType()));
		_msgWmSyncInfo.obstacleObjectID.push_back(it->getID().uniqueID);
	}
}

void worldModelSyncInfoUpdaterROS::setRobotLocation(robotClass_t robotLocation)
{
	_msgWmSyncInfo.locationTimestamp = robotLocation.getTimestamp();
	_msgWmSyncInfo.locationX = robotLocation.getX();
	_msgWmSyncInfo.locationY = robotLocation.getY();
	_msgWmSyncInfo.locationTheta = robotLocation.getTheta();
	_msgWmSyncInfo.locationVx = robotLocation.getVX();
	_msgWmSyncInfo.locationVy = robotLocation.getVY();
	_msgWmSyncInfo.locationVtheta = robotLocation.getVTheta();
}

void worldModelSyncInfoUpdaterROS::setRobotStatus(robotStatusType status)
{
	if(status == robotStatusType::INPLAY)
	{
		_msgWmSyncInfo.ownRobotIsActive = true;
	}
	else
	{
		_msgWmSyncInfo.ownRobotIsActive = false;
	}
}

void worldModelSyncInfoUpdaterROS::sendPacket()
{
	try
	{
		_pWmInfo.publish(_msgWmSyncInfo);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
