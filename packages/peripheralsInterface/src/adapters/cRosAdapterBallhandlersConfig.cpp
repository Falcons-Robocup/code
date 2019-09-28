 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterBallhandlersConfig.cpp
 *
 *  Created on: Feb 11, 2016
 *	  Author: Tim Kouters
 */

#include <exception>

#include "FalconsCommon.h"
#include "tracing.hpp"

#include "int/adapters/cRosAdapterBallhandlersConfig.hpp"

using std::cout;
using std::endl;
using std::exception;

/*
 * Class implementation
 */
cRosAdapterBallhandlersConfig::cRosAdapterBallhandlersConfig(PeripheralsInterfaceData &piData):
		_srv(ros::NodeHandle("~/ballhandlers")), _piData(piData) {
	TRACE(">");

	TRACE("<");
}

cRosAdapterBallhandlersConfig::~cRosAdapterBallhandlersConfig() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterBallhandlersConfig::initialize() {
	TRACE(">");

	/* Bind the reconfiguration function */
	dynamic_reconfigure::Server<peripheralsInterface::ballhandlersConfig>::CallbackType f;
	f = boost::bind(&cRosAdapterBallhandlersConfig::cRosAdapterBallhandlersConfig_cb, this, _1, _2);

	/* Start the server with the callback; this will automatically reconfigure the settings. */
	_srv.setCallback(f);

	TRACE("<");
}

void cRosAdapterBallhandlersConfig::cRosAdapterBallhandlersConfig_cb(peripheralsInterface::ballhandlersConfig &config, uint32_t level) {
	TRACE(">");

	BallhandlerBoardSettings settings;
	settings.ballhandlerPlotEnabled = config.ballhandlerPlotEnabled;
	settings.pid.p = config.Kp;
	settings.pid.i = config.Ki;
	settings.pid.d = config.Kd;
	settings.pid.iTh = config.iTh;
	settings.pid.iMax = config.iMax;
	settings.anglePid.p = config.Ang_Kp;
	settings.anglePid.i = config.Ang_Ki;
	settings.anglePid.d = config.Ang_Kd;
	settings.anglePid.iTh = 0;
	settings.maxPwmValue = config.PwmMax;
	settings.maxPwmStepValue = config.PwmMaxDeltaSize;

	_piData.getLeftBallhandlerBoard().setSettings(settings);
	_piData.getRightBallhandlerBoard().setSettings(settings);

	TRACE("<");
}
