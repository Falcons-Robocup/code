 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterIMUConfig.cpp
 *
 *  Created on: 23 feb. 2016
 *      Author: Edwin Schreuder
 */

#include "int/adapters/cRosAdapterIMUConfig.hpp"

#include <ros/node_handle.h>
#include <dynamic_reconfigure/server.h>
#include <peripheralsInterface/compassConfig.h>

#include "FalconsCommon.h"
#include <cDiagnosticsEvents.hpp>

#include "int/IMU/IMU.hpp"

/*
 * Class implementation
 */
cRosAdapterIMUConfig::cRosAdapterIMUConfig(IMU &imu) : _imu(imu) {
	TRACE(">");

	// compass workaround for playing direction
	_flipCompass = false;
	int playingDirection;
	ros::param::get("globalConfig/playingDirection", playingDirection);
	TRACE("playingDirection=%d", playingDirection);

	if (playingDirection == 1) {
		TRACE("applying compass flip workaround for playing direction");
		_flipCompass = true;
		// TODO: make this reconfigurable? for now: init time
	}

	dynamic_reconfigure::Server<peripheralsInterface::compassConfig>::CallbackType f;

	f = boost::bind(&cRosAdapterCompassConfig_cb, _1, _2, this);

	_server.setCallback(f);

	TRACE("<");
}

void cRosAdapterIMUConfig::cRosAdapterCompassConfig_cb(peripheralsInterface::compassConfig &config, uint32_t level, cRosAdapterIMUConfig *classRef) {
	TRACE(">");

	float newHomeGoalAngle = config.homeGoalAngle;
	if (classRef->_flipCompass) {
		newHomeGoalAngle = fmod(newHomeGoalAngle + 180.0, 360.0);
		TRACE_INFO("applying compass flip workaround for playing direction to homegoalAngle (%.1f)", newHomeGoalAngle);
	}

	cout << "Setting homeGoal angle to " << newHomeGoalAngle << "." << endl;

	classRef->_imu.setHomeGoalAngle(newHomeGoalAngle);

	TRACE("<");
}

void cRosAdapterIMUConfig::cRosAdapterIMUConfig_cb(peripheralsInterface::compassConfig &config, uint32_t level, cRosAdapterIMUConfig *classRef) {
	TRACE(">");

	// TODO: Add configuration of IMU parameters.

	TRACE("<");
}
