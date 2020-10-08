 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
