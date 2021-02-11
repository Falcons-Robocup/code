// Copyright 2016 Edwin (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Yocto3DModule.hpp
 *
 *  Created on: 16 feb. 2016
 *      Author: Edwin Schreuder
 */

#ifndef YOCTO3DMODULE_HPP_
#define YOCTO3DMODULE_HPP_

#include <iostream>
#include <string>

#include "tf/LinearMath/Vector3.h"

#include "yocto_accelerometer.h"
#include "yocto_compass.h"
#include "yocto_gyro.h"
#include "yocto_magnetometer.h"
#include "yocto_refframe.h"
#include "yocto_tilt.h"

#include "YoctoModule.hpp"

using namespace std;
using tf::Vector3;

struct Yocto3DData {
	Vector3 accelerometer_vector;
	Vector3 magnetometer_vector;
	Vector3 gyro_vector;
	Vector3 attitude_vector;
	double compass_angle;
	double pitch_angle;
	double roll_angle;
};

class Yocto3DModule : public YoctoModule
{
	private:
		Y_MOUNTPOSITION determineMountPosition(Vector3 cross_vector, Vector3 &reference_vector);
		Y_MOUNTORIENTATION determineMountOrientation(double angle);

	protected:
		YAccelerometer * accelerometer;
		YMagnetometer * magnetometer;
		YGyro * gyro;
		YCompass * compass;
		YRefFrame * ref_frame;
		YTilt * pitch;
		YTilt * roll;

	public:
		Yocto3DModule(const string module_name, const string hub = "usb");

		Yocto3DData getYocto3DData();
		Vector3 getAccelerometerVector();
		Vector3 getMagnetometerVector();
		Vector3 getGyroVector();
		Vector3 getAttitudeVector();
		double getCompassAngle();
		double getPitchAngle();
		double getRollAngle();

		void setGravityCancellationOn();
		void setGravityCancellationOff();

		void setReferenceFrame(Vector3 x_vector, Vector3 y_vector);
		// Set the compass angle offset (in degrees)
		void setCompassAngleOffset(double offset);
};

#endif /* YOCTO3DMODULE_HPP_ */
