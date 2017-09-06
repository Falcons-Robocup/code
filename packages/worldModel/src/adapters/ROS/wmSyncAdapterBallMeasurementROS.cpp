 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * wmSyncAdapterBallMeasurementROS.cpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/ROS/wmSyncAdapterBallMeasurementROS.hpp"

#include "int/facilities/ROSConvert.hpp"

#include "ext/WorldModelNames.h"

#include "cDiagnosticsEvents.hpp"

wmSyncAdapterBallMeasurementROS::wmSyncAdapterBallMeasurementROS()
{

}

wmSyncAdapterBallMeasurementROS::~wmSyncAdapterBallMeasurementROS()
/*
 * There is no theory of evolution,
 * just a list of creatures Chuck Norris allows to live
 */
{

}

void wmSyncAdapterBallMeasurementROS::InitializeROS()
{
	try
	{
		_hROS.reset(new ros::NodeHandle());

		_srvRemoteBalls = _hROS->advertiseService(
                WorldModelInterface::s_set_remote_ball_location,
                &wmSyncAdapterBallMeasurementROS::set_remote_ball_location_cb, this);

	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool wmSyncAdapterBallMeasurementROS::set_remote_ball_location_cb(
        		worldModel::set_remote_ball_location::Request &req,
        		worldModel::set_remote_ball_location::Response &resp)
{
	try
	{
		std::vector<ballMeasurementType> balls;

		for(size_t i = 0; i < req.azimuth.size(); i++)
		{
			ballMeasurementType ball;

			ball.setConfidence(req.confidence.at(i));
			ball.setTimestamp(req.timestamp.at(i));
			ball.setID(uniqueWorldModelID(
					req.robotID.at(i),
					req.objectID.at(i)));
			ball.setCameraType(convertCameraType(req.cameraType.at(i)));
			ball.setSphericalCoords(
					req.azimuth.at(i),
					req.elevation.at(i),
					req.radius.at(i));
			ball.setCameraOffset(
					req.cameraX.at(i),
					req.cameraY.at(i),
					req.cameraZ.at(i),
					req.cameraPhi.at(i));

			balls.push_back(ball);
		}

		notify(balls);

		return true;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
