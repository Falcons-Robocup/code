 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterCompassConfig.cpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Tim Kouters
 */

#include "int/adapters/cRosAdapterCompassConfig.hpp"

#include <boost/any.hpp>
#include <dynamic_reconfigure/server.h>
#include <stdexcept>
#include <string>

#include "FalconsCommon.h"
#include "int/config/cCompassConfig.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::exception;
using std::string;

/*
 * Class implementation
 */
cRosAdapterCompassConfig::cRosAdapterCompassConfig() {
	TRACE(">");

	try {
		loadConfiguration();
	}
	catch (exception &e) {
		throw e;
	}

	TRACE("<");
}

cRosAdapterCompassConfig::~cRosAdapterCompassConfig() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterCompassConfig::loadConfiguration() {
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

	try {
		/* Bind the reconfiguration function */
		f = boost::bind(&cRosAdapterCompassConfig_cb, _1, _2, this);
		srv.setCallback(f);

		peripheralsInterface::compassConfig config;
		cout << "INFO    : Reading worldModel parameters from ROS parameter server" << endl;
		while (!ros::param::get("compass/homeGoalAngle", config.homeGoalAngle)) {
			cout << "INFO    : Compass parameters not loaded yet??" << endl;
			sleep(5);
		}
		cout << "INFO    : Homegoal angle " << config.homeGoalAngle << std::endl;

		cRosAdapterCompassConfig_cb(config, 0, this);
	}
	catch (exception &e) {
		throw e;
	}

	TRACE("<");
}

void cRosAdapterCompassConfig::cRosAdapterCompassConfig_cb(peripheralsInterface::compassConfig &config, uint32_t level, cRosAdapterCompassConfig *classRef) {
	TRACE(">");

	try {
		float newHomeGoalAngle = config.homeGoalAngle;
		if (classRef->_flipCompass) {
			newHomeGoalAngle = fmod(newHomeGoalAngle + 180.0, 360.0);
			TRACE("applying compass flip workaround for playing direction to homegoalAngle (%.1f)", newHomeGoalAngle);
		}

		cCompassConfig::getInstance().setHomeGoalAngle(newHomeGoalAngle);

	}
	catch (exception &e) {
		cerr << "ERROR   : Error occured at reconfiguring Compass" << e.what() << endl;
		throw e;
	}

	TRACE("<");
}
