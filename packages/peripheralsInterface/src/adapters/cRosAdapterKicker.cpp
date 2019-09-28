 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include <cDiagnostics.hpp>
#include <FalconsCommon.h>
#include "tracing.hpp"

#include "../../include/ext/peripheralsInterfaceNames.hpp"

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

	/* Bind the reconfiguration function */
	auto callback = bind(&cRosAdapterKicker::cbConfig, this, _1, _2);

	/* Start the server with the callback; this will automatically reconfigure the settings. */
	configServer.setCallback(callback);


	TRACE("<");
}

void cRosAdapterKicker::cbConfig(peripheralsInterface::kickerConfig &config, uint32_t level)
{
	TRACE(">");

	kicker.setLeverSpeed(config.leverSpeed);

	TRACE("<");
}
