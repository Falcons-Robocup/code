 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cROSAdapter.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Tim Kouters
 */

#include "adapters/cROSAdapter.hpp"

#include <stdexcept>

#include "worldModel/set_own_ball_location.h"
#include "WorldModelNames.h"
#include "FalconsCommon.h"
#include "timeConvert.hpp"
#include <cDiagnosticsEvents.hpp>

using std::exception;

cROSAdapter::cROSAdapter(cDiagAdapter *diagAdapter)
{
    _diagAdapter = diagAdapter;
	double ratio = 720.0 * 1280.0;

	_lutDistance.insert(31500.0/ ratio, 0.5);
	_lutDistance.insert(8250.0 / ratio, 1.0);
	_lutDistance.insert(1785.0 / ratio, 2.0);
	_lutDistance.insert(720.0 / ratio, 3.0);
	_lutDistance.insert(370.0 / ratio, 4.0);
	_lutDistance.insert(230.0 / ratio, 5.0);
	_lutDistance.insert(155.0 / ratio, 6.0);
	_lutDistance.insert(110.0 / ratio, 7.0);
	_lutDistance.insert(80.0 / ratio, 8.0);
	_lutDistance.insert(65.0 / ratio, 9.0);
	_lutDistance.insert(50.0 / ratio, 10.0);
}

cROSAdapter::~cROSAdapter()
{

}

void cROSAdapter::initializeAdapter()
{
	TRACE(">");
    _hROS.reset(new ros::NodeHandle());
    ros::service::waitForService(WorldModelInterface::s_set_own_ball_location);
    _sSetFrontCamera = _hROS->serviceClient<worldModel::set_own_ball_location>(WorldModelInterface::s_set_own_ball_location, true);
    TRACE("<");
}

void cROSAdapter::setBall(const std::vector<ballPositionType> &balls)
{
    //TRACE("received %d balls", (int)balls.size());
	worldModel::set_own_ball_location srvFrontCam;

    double timestamp = getTimeNow(); 
    rosMsgs::s_camera_type camType;
    camType.camera_type = rosMsgs::s_camera_type::TYPE_FRONT;

	try
	{
		/*
		 * convert to worldModel ball input structure
		 */
        // convert arguments to regular spherical coordinates (azimuth, elevation, radius)
		for(auto it = balls.begin(); it != balls.end(); it++)
		{
		    float size = it->getSize();
		    float angle = it->getAngle();
		    float radius = it->getRadiusAngle();
		    float distance = _lutDistance.linear_interp(size);
            float azimuth = cos(angle - ANGLE_OFFSET) * radius;
            float elevation = -sin(angle - ANGLE_OFFSET) * radius;

            srvFrontCam.request.azimuth.push_back(azimuth);
            srvFrontCam.request.elevation.push_back(elevation);
            srvFrontCam.request.radius.push_back(distance);

			srvFrontCam.request.confidence.push_back(it->getConfidence());

            srvFrontCam.request.timestamp.push_back(timestamp);
            srvFrontCam.request.cameraType.push_back(camType);

			srvFrontCam.request.relativeCameraX.push_back(0.0);
			srvFrontCam.request.relativeCameraY.push_back(CAMERA_FRONT_OFFSET);
			srvFrontCam.request.relativeCameraZ.push_back(CAMERA_MOUNTING_HEIGHT);
			srvFrontCam.request.relativeCameraPhi.push_back(CAMERA_FRONT_PHI);
            //TRACE("ang=%5.2f rad=%5.2f dist=%5.2f az=%5.2f el=%5.2f", angle, radius, distance, azimuth, elevation);
		}

		if(!_sSetFrontCamera.call(srvFrontCam))
		{
			TRACE_ERROR("Failed to set front camera ball");
			initializeAdapter();
		}
		
		// traditional logging, plot as cyan balls in window
        //_diagAdapter->setBall(project_angle_0_2pi(angle + ANGLE_OFFSET), distance*radius, distance);
	}
	catch (exception &e)
	{
		throw e;
	}
}

