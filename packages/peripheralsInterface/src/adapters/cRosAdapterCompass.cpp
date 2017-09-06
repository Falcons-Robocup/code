 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterCompass.cpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Tim Kouters
 */


#include "int/adapters/cRosAdapterCompass.hpp"

#include <stdexcept>

#include "int/compass/cCompass.hpp"
#include "ext/peripheralInterfaceNames.hpp"

using std::exception;
using std::runtime_error;

cRosAdapterCompass::cRosAdapterCompass(cCompass *compass, boost::function<void(double)> &callback) : _callback(callback) {
	TRACE(">");

	try {
		if (compass == NULL)
		{
			throw runtime_error("compass cannot be NULL");
		}

		_compass = compass;
		_srvGetCompass = _n.advertiseService(peripheralInterfaceServiceNames::s_get_compass, &cRosAdapterCompass::cbCompass, this);
		_srvGetRawCompass = _n.advertiseService(peripheralInterfaceServiceNames::s_get_raw_compass, &cRosAdapterCompass::cbCompassRaw, this);
	}
	catch (exception &e) {
		throw e;
	}

	TRACE("<");
}

cRosAdapterCompass::~cRosAdapterCompass() {
	TRACE(">");

	TRACE("<");
}

bool cRosAdapterCompass::cbCompass(peripheralsInterface::s_compass::Request& request, peripheralsInterface::s_compass::Response& response) {
//	TRACE(">");

	bool retVal = true;

	try {
		_compass->getCompassAngle(response.theta);
		_callback(response.theta);
	}
	catch (exception &e) {
		throw e;
	}

//	TRACE("<");

	return retVal;
}

bool cRosAdapterCompass::cbCompassRaw(peripheralsInterface::s_compass_raw::Request& request, peripheralsInterface::s_compass_raw::Response& response) {
//	TRACE(">");

	bool retVal = true;

	try {
		_compass->getRawCompassAngle(response.theta);
	}
	catch (exception &e) {
		throw e;
	}

//	TRACE("<");

	return retVal;
}
