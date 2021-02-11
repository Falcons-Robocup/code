// Copyright 2016-2020 Edwin (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IMU.cpp
 *
 *  Created on: Feb 23, 2016
 *      Author: Edwin Schreuder
 */

#include <exception>

#include "int/IMU/IMU.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"

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
