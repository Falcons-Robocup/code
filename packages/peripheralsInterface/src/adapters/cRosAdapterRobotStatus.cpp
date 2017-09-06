 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterRobotStatus.cpp
 *
 *  Created on: Apr 27, 2017
 *      Author: Edwin Schreuder
 */

#include "int/adapters/cRosAdapterRobotStatus.hpp"

#include <iostream>
#include <string>

#include <cDiagnosticsEvents.hpp>
#include <FalconsCommon.h>
#include <WorldModelNames.h>

using std::cerr;
using std::endl;
using std::string;

cRosAdapterRobotStatus::cRosAdapterRobotStatus() {
	TRACE(">");

	TRACE("<");
}

cRosAdapterRobotStatus::~cRosAdapterRobotStatus() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterRobotStatus::initialize() {
	TRACE(">");

	connectToWorldModel(0.0);

	TRACE("<");
}

void cRosAdapterRobotStatus::reInitialize() {
	TRACE(">");

	connectToWorldModel(0.1);

	TRACE("<");
}

void cRosAdapterRobotStatus::connectToWorldModel(double timeout) {

	TRACE(">");

	bool connected = ros::service::waitForService(
			WorldModelInterface::s_set_own_robot_status,
			ros::Duration(timeout));

	if (connected) {
		_srvSetRobotStatus = _n.serviceClient<worldModel::set_own_robot_status>(WorldModelInterface::s_set_own_robot_status, true);
	}
	else {
//		TRACE_ERROR("Could not initialize peripheralsInterface RobotStatus subscription!");
//		cerr << "Could not initialize peripheralsInterface RobotStatus subscription." << endl;
	}

	TRACE("<");
}

void cRosAdapterRobotStatus::setRobotStatus(bool inPlay) {
	worldModel::set_own_robot_status robotStatus;
	string logtext;

	if (inPlay) {
		logtext = "Inplay is set to true.";
		robotStatus.request.robotStatus.status_type = robotStatus.request.robotStatus.TYPE_INPLAY;
	}
	else {
		logtext = "Inplay is set to false.";
		robotStatus.request.robotStatus.status_type = robotStatus.request.robotStatus.TYPE_OUTOFPLAY;
	}

	if (!_srvSetRobotStatus.call(robotStatus))
	{
//		TRACE_ERROR("Failed to send robot status to worldModel.");
//		cerr << "Failed to send robot status to worldModel." << endl;
		reInitialize();
	}
	else {
		TRACE(logtext.c_str());
	}
}
