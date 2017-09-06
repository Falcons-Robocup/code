 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * IMU.cpp
 *
 *  Created on: Feb 23, 2016
 *      Author: Edwin Schreuder
 */

#include <exception>

#include "int/IMU/IMU.hpp"

#include "FalconsCommon.h"

#include "int/IMU/Yocto3DModule.hpp"

using std::runtime_error;
using tf::Vector3;

IMU::IMU() : _module(Yocto3DModule(""))
{
    _module.setReferenceFrame(Vector3(0, 0, -1), Vector3(1, 0, 0));
    _module.setCompassAngleOffset(0.0);
    _homeGoalAngle = 0;
}

double IMU::getCompassAngle()
{
	double angle;

    angle = fmod(360.0 + _homeGoalAngle - getRawCompassAngle(), 360.0);

    TRACE("angle = %6.2f", angle);
	return angle;
}

double IMU::getRawCompassAngle()
{
	double angle;

	if (_module.isOnline()) {
		/*
		 * The compass angle is now defined as the integrated gyro value, in combination
		 * with a filtered offset reference sourced by the magnetometer.
		 *
		 * To obtain the raw compass value, the module.getCompassAngle() function could be used.
		 */

		// getAttitudeVector returns a value between -180 and 180 degrees.
//		angle = _module.getAttitudeVector().getZ();
		// getCompassAngle returns a value between 0 and 360 degrees.
		angle = _module.getCompassAngle();
		if (angle < 0)
		{
			angle += 360;
		}
		else if (angle > 360)
		{
			angle -= 360;
		}
	}
	else {
		throw(runtime_error(""));
	}

    TRACE("angle = %6.2f", angle);
	return angle;
}

Vector3 IMU::getVector()
{
	return _module.getAttitudeVector();
}

double IMU::getHomeGoalAngle()
{
	return _homeGoalAngle;
}

void IMU::setHomeGoalAngle(double angle)
{
	cout << "Homegoalangle: " << angle << ", module: " << _module.getSerialNumber() << "." << endl;
	TRACE("setHomeGoalAngle(%.1f)", angle);
	_homeGoalAngle = angle;
}
