 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Yocto3DModule.cpp
 *
 *  Created on: 16 feb. 2016
 *      Author: Edwin Schreuder
 */

#include <exception>
#include <iostream>
#include <cmath>
#include <string>

#include "int/IMU/Yocto3DModule.hpp"

#include "tf/LinearMath/Vector3.h"

#include "yocto_accelerometer.h"
#include "yocto_compass.h"
#include "yocto_gyro.h"
#include "yocto_magnetometer.h"
#include "yocto_refframe.h"
#include "yocto_tilt.h"

#include "int/IMU/YoctoModule.hpp"

using namespace std;
using tf::Vector3;

YAccelerometer * accelerometer;
YMagnetometer * magnetometer;
YGyro * gyro;
YCompass * compass;
YRefFrame * refFrame;
YTilt * pitch;
YTilt * roll;

Yocto3DModule::Yocto3DModule(const string module_name, const string hub) : YoctoModule(module_name, "Yocto-3D", hub)
{
	accelerometer = YAccelerometer::FindAccelerometer(getSerialNumber() + ".accelerometer");
	cout << "Found accelerometer (" << accelerometer->describe() << ")." << endl;

	magnetometer = YMagnetometer::FindMagnetometer(getSerialNumber() + ".magnetometer");
	cout << "Found magnetometer (" << magnetometer->describe() << ")." << endl;

	gyro = YGyro::FindGyro(getSerialNumber() + ".gyro");
	cout << "Found gyro (" << gyro->describe() << ")." << endl;

	compass = YCompass::FindCompass(getSerialNumber() + ".compass");
	cout << "Found compass (" << compass->describe() << ")." << endl;

	ref_frame = YRefFrame::FindRefFrame(getSerialNumber() + ".refFrame");
	cout << "Found ref_frame (" << ref_frame->describe() << ")." << endl;

	pitch = YTilt::FindTilt(getSerialNumber() + ".tilt1");
	cout << "Found pitch (" << pitch->describe() << ")." << endl;

	roll = YTilt::FindTilt(getSerialNumber() + ".tilt2");
	cout << "Found roll (" << roll->describe() << ")." << endl;
}

Yocto3DData Yocto3DModule::getYocto3DData()
{
	Yocto3DData data;

	data.accelerometer_vector = getAccelerometerVector();
	data.magnetometer_vector = getMagnetometerVector();
	data.gyro_vector = getGyroVector();
	data.attitude_vector = getAttitudeVector();
	data.compass_angle = getCompassAngle();
	data.pitch_angle = getPitchAngle();
	data.roll_angle = getPitchAngle();

	return (data);
}

tf::Vector3 Yocto3DModule::getAccelerometerVector()
{
	Vector3 accelerometer_vector;

	accelerometer_vector.setX(accelerometer->xValue());
	accelerometer_vector.setY(accelerometer->yValue());
	accelerometer_vector.setZ(accelerometer->zValue());

	return (accelerometer_vector);
}

Vector3 Yocto3DModule::getMagnetometerVector()
{
	Vector3 magnetometer_vector;

	magnetometer_vector.setX(magnetometer->xValue());
	magnetometer_vector.setY(magnetometer->yValue());
	magnetometer_vector.setZ(magnetometer->zValue());

	return (magnetometer_vector);
}

Vector3 Yocto3DModule::getGyroVector()
{
	Vector3 gyro_vector;

	gyro_vector.setX(gyro->xValue());
	gyro_vector.setY(gyro->yValue());
	gyro_vector.setZ(gyro->zValue());

	return (gyro_vector);
}

Vector3 Yocto3DModule::getAttitudeVector()
{
	Vector3 attitude_vector;

	attitude_vector.setX(gyro->get_roll());
	attitude_vector.setY(gyro->get_pitch());
	attitude_vector.setZ(gyro->get_heading());

	return (attitude_vector);
}

double Yocto3DModule::getCompassAngle()
{
	return (compass->magneticHeading());
}

double Yocto3DModule::getPitchAngle()
{
	return (pitch->currentValue());
}

double Yocto3DModule::getRollAngle()
{
	return (roll->currentValue());
}

void Yocto3DModule::setGravityCancellationOn()
{
	accelerometer->setGravityCancellation(Y_GRAVITYCANCELLATION_ON);
}

void Yocto3DModule::setGravityCancellationOff()
{
	accelerometer->setGravityCancellation(Y_GRAVITYCANCELLATION_OFF);
}

void Yocto3DModule::setReferenceFrame(Vector3 x_vector, Vector3 y_vector)
{
	Y_MOUNTPOSITION position;
	Y_MOUNTORIENTATION orientation;

	/*
	 * The reference frame of the Yocto3D module needs to be explicitly set in software
	 * in order for the results to be of any value. The supplier assumes that the module is
	 * mounted orthogonal to the global cartesian reference frame of the object. This gives
	 * 24 possiblities for the device to be mounted. The x and y inputs are according to
	 * the markings on the Yocto3D module.
	 *
	 * The input vectors should be aligned with either the x, y or z axis of the robot.
	 */

	// Currently, the module can only be configured orthogonal to the axes of the
	// standard Cartesian reference frame.
	size_t x_nonzero_components = 0;
	size_t y_nonzero_components = 0;
	if (x_vector.x() != 0) x_nonzero_components++;
	if (x_vector.y() != 0) x_nonzero_components++;
	if (x_vector.z() != 0) x_nonzero_components++;
	if (y_vector.x() != 0) y_nonzero_components++;
	if (y_vector.y() != 0) y_nonzero_components++;
	if (y_vector.z() != 0) y_nonzero_components++;

	// Check if the vectors are aligned with one of the axes and are orthogonal to each other.
	if ((x_nonzero_components != 1) || (y_nonzero_components != 1) || (x_vector.dot(y_vector) != 0))
	{
		throw runtime_error("Vectors are not orthogonal and/or aligned with Cartesian axes.");
	}

	// Calculate the cross product and define module position.
	Vector3 cross_vector = x_vector.cross(y_vector);
	Vector3 reference_vector;
	position = determineMountPosition(cross_vector, reference_vector);

	// Calculate the sum of the vector and the angle between the result and the reference vector.
	double angle = (x_vector + y_vector).angle(reference_vector);
	orientation = determineMountOrientation(angle);

	cout << "IMU: sending mount position (position = " << position << ", orientation = " << orientation << ")." << endl;

	// Propagate the mount position to the Yocto-3D module.
	ref_frame->set_mountPosition(position, orientation);
}

Y_MOUNTPOSITION Yocto3DModule::determineMountPosition(Vector3 cross_vector, Vector3 &reference_vector)
{
	Y_MOUNTPOSITION position;

	/*
	 * Based on the cross vector, we can now define the position (left, right, bottom, top
	 * front, rear) of the module.
	 * If the cross vector aligns with -Y, the module is in the LEFT position.
	 * If the cross vector aligns with +Y, the module is in the RIGHT position.
	 * If the cross vector aligns with +Z, the module is in the BOTTOM position
	 * If the cross vector aligns with -Z, the module is in the TOP position.
	 * If the cross vector aligns with -X, the module is in the FRONT position.
	 * If the cross vector aligns with +X, the module is in the REAR position.
	 *
	 * The reference vector aligns with the input vectors when the module is in the zero degree
	 * (12 'o clock) position.
	 */

	if (cross_vector.z() > 0)
	{
		position = Y_MOUNTPOSITION_BOTTOM;
		reference_vector.setValue(1, 1, 0);
	}
	else if (cross_vector.z() < 0)
	{
		position = Y_MOUNTPOSITION_TOP;
		reference_vector.setValue(-1, 1, 0);
	}
	else if (cross_vector.x() < 0)
	{
		position = Y_MOUNTPOSITION_FRONT;
		reference_vector.setValue(0, 1, 1);
	}
	else if (cross_vector.x() > 0)
	{
		position = Y_MOUNTPOSITION_REAR;
		reference_vector.setValue(0, -1, 1);
	}
	else if (cross_vector.y() > 0)
	{
		position = Y_MOUNTPOSITION_RIGHT;
		reference_vector.setValue(1, 0, 1);
	}
	else if (cross_vector.y() < 0)
	{
		position = Y_MOUNTPOSITION_LEFT;
		reference_vector.setValue(-1, 0, 1);
	}
	else {
		throw runtime_error("Could not determine the mount position of the module.");
	}

	return position;
}

Y_MOUNTORIENTATION Yocto3DModule::determineMountOrientation(double angle)
{
	Y_MOUNTORIENTATION orientation;

	/*
	 * Based on the angle between the reference vector and this vector, we can now define
	 * the orientation (12 'o clock, 3 'o clock, 6 'o clock and 9 'o clock) of the module
	 * The reference vector aligns with the input vectors when the module is in the zero degree
	 * (12 'o clock) position.
	 *
	 * If -0.25 * pi < angle <=  0.25 * pi: 12 'o clock
	 * If  0.25 * pi < angle <=  0.75 * pi:  9 'o clock
	 * If  0.75 * pi < angle <= -0.75 * pi:  6 'o clock
	 * If -0.75 * pi < angle <= -0.25 * pi:  3 'o clock
	 */

	if ((angle > (-0.75 * M_PI)) && (angle <= (-0.25 * M_PI)))
	{
		orientation = Y_MOUNTORIENTATION_THREE;
	}
	else if ((angle > (-0.25 * M_PI)) && (angle <= (0.25 * M_PI)))
	{
		orientation = Y_MOUNTORIENTATION_TWELVE;
	}
	else if ((angle > (0.25 * M_PI)) && (angle <= (0.75 * M_PI)))
	{
		orientation = Y_MOUNTORIENTATION_NINE;
	}
	else {
		orientation = Y_MOUNTORIENTATION_SIX;
	}

	return orientation;

}

void Yocto3DModule::setCompassAngleOffset(double offset)
{
	ref_frame->setBearing(offset/180 * M_PI);
}
