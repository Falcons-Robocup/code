 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterKicker.cpp
 *
 *  Created on: Apr 25, 2017
 *      Author: Edwin Schreuder
 */

#include "int/adapters/cRosAdapterKicker.hpp"

#include <exception>
#include <functional>

#include <cDiagnosticsEvents.hpp>
#include <FalconsCommon.h>

#include "ext/peripheralInterfaceNames.hpp"

using std::bind;
using std::exception;

cRosAdapterKicker::cRosAdapterKicker(Kicker& kicker) :
	kicker(kicker)
{
	TRACE(">");

	TRACE("<");
}

cRosAdapterKicker::~cRosAdapterKicker() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterKicker::initialize() {
	TRACE(">");

	setKickPositionService = nodeHandle.advertiseService(
			peripheralInterfaceServiceNames::setKickPosition, &cRosAdapterKicker::cbSetKickPosition, this);
	setKickSpeedService = nodeHandle.advertiseService(
			peripheralInterfaceServiceNames::setKickSpeed, &cRosAdapterKicker::cbSetKickSpeed, this);
	homeKickerService = nodeHandle.advertiseService(
			peripheralInterfaceServiceNames::homeKicker, &cRosAdapterKicker::cbHomeKicker, this);

	/* Bind the reconfiguration function */
	auto callback = bind(&cRosAdapterKicker::cbConfig, this, _1, _2);

	/* Start the server with the callback; this will automatically reconfigure the settings. */
	configServer.setCallback(callback);


	TRACE("<");
}

bool cRosAdapterKicker::cbSetKickPosition(peripheralsInterface::s_peripheralsInterface_setKickPosition::Request& request, peripheralsInterface::s_peripheralsInterface_setKickPosition::Response& response) {
	TRACE("> kick position: %f", request.kick_position);

	try {
		cout << "Setting height." << endl;
		kicker.setHeight(request.kick_position);

		cout << "Issuing move." << endl;
		kicker.move();
	}
	catch(exception &e) {
		cout << e.what() << endl;
		throw(e);
	}

	TRACE("<");

	return true;
}

bool cRosAdapterKicker::cbSetKickSpeed(peripheralsInterface::s_peripheralsInterface_setKickSpeed::Request& request, peripheralsInterface::s_peripheralsInterface_setKickSpeed::Response& response) {
	TRACE("> kick speed: %f", request.kick_speed);

	try {
		cout << "Setting shoot power." << endl;
		kicker.setShootPower(request.kick_speed);

		cout << "Issuing shoot." << endl;
		kicker.shoot();
	}
	catch(exception &e) {
		cout << e.what() << endl;
		throw(e);
	}

	TRACE("<");

	return true;
}

bool cRosAdapterKicker::cbHomeKicker(peripheralsInterface::s_homeKicker::Request& request, peripheralsInterface::s_homeKicker::Response& response) {
	TRACE(">");

	kicker.home();

	TRACE("<");

	return true;
}

void cRosAdapterKicker::cbConfig(peripheralsInterface::kickerConfig &config, uint32_t level)
{
	TRACE(">");

	kicker.setLeverSpeed(config.leverSpeed);

	TRACE("<");
}
