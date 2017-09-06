 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterIMU.cpp
 *
 *  Created on: 23 feb. 2016
 *      Author: Edwin Schreuder
 */

#include <exception>

#include "tf/LinearMath/Vector3.h"

#include "FalconsCommon.h"
#include "cDiagnosticsEvents.hpp"

#include "rosMsgs/t_compass.h"

#include "ext/peripheralInterfaceNames.hpp"

#include "int/adapters/cRosAdapterIMU.hpp"

#include "int/IMU/IMU.hpp"

using std::runtime_error;

using tf::Vector3;

cRosAdapterIMU::cRosAdapterIMU(IMU &imu, boost::function<void(double)> &callback) : _imu(imu), _callback(callback) {
	TRACE(">");

	_srvGetCompass = _n.advertiseService(peripheralInterfaceServiceNames::s_get_compass, &cRosAdapterIMU::cbGetCompass, this);
	_msgCompass = _n.advertise<rosMsgs::t_compass>(peripheralInterfaceTopicNames::topicCompass, 20);

	_fetchCompassThread = boost::thread(boost::bind(&cRosAdapterIMU::cbFetchCompassFromIMU, this));

	TRACE("<");
}

void cRosAdapterIMU::cbFetchCompassFromIMU() {
	while (true) {
		Vector3 vector;

		try {
			_value = _imu.getCompassAngle();
			vector = _imu.getVector();
		}
		catch (exception &e) {
			TRACE_ERROR("Unable to get compass values from IMU: ", e.what());
		}

		rosMsgs::t_compass compassMsg;
		compassMsg.theta = _value;
		compassMsg.heading = vector.getZ();
		compassMsg.roll = vector.getX();
		compassMsg.pitch = vector.getY();
		_msgCompass.publish(compassMsg);
		_callback(_value);

//		TRACE("< compassMsg.theta = %6.2f", compassMsg.theta);

		boost::this_thread::sleep_for( boost::chrono::milliseconds(100) );
	}
}

bool cRosAdapterIMU::cbGetCompass(peripheralsInterface::s_compass::Request& request, peripheralsInterface::s_compass::Response& response) {
	bool retVal = true;

	response.theta = _value;
	TRACE("response.theta = %6.2f", response.theta);

	return retVal;
}
