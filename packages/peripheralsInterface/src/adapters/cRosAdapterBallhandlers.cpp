 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterBallhandlers.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: Tim Kouters
 */
#include <stdexcept>

#include <FalconsCommon.h>

#include "int/adapters/cRosAdapterBallhandlers.hpp"

#include "ext/peripheralInterfaceNames.hpp"

using std::cout;
using std::endl;
using std::exception;

static rosMsgs::t_bh_pid_params boardDataToPidParameters(BallhandlerBoardDataOutput boardDataOutput);

cRosAdapterBallhandlers::cRosAdapterBallhandlers(PeripheralsInterfaceData& piData) :
		_piData(piData) {
	TRACE(">");

	TRACE("<");
}

cRosAdapterBallhandlers::~cRosAdapterBallhandlers() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterBallhandlers::initialize() {
	TRACE(">");

	_tBhPidParameters = _n.advertise<rosMsgs::t_bh_pid_params>(
			peripheralInterfaceTopicNames::topicBHPIDParameters, 20.0);

	_srvGetAngle = _n.advertiseService(peripheralInterfaceServiceNames::getBallHandlersAngle, &cRosAdapterBallhandlers::cbGetAngle, this);
	_srvSetAngle = _n.advertiseService(peripheralInterfaceServiceNames::setBallHandlersAngle, &cRosAdapterBallhandlers::cbSetAngle, this);
	_srvEnableBallhandlers = _n.advertiseService(peripheralInterfaceServiceNames::enableBallHandlers, &cRosAdapterBallhandlers::cbEnableBallhandlers, this);
	_srvDisableBallhandlers = _n.advertiseService(peripheralInterfaceServiceNames::disableBallHandlers, &cRosAdapterBallhandlers::cbDisableBallhandlers, this);

	TRACE("<");
}

void cRosAdapterBallhandlers::publishPidParameters() {
	rosMsgs::t_bh_pid_params bhPidParamsMsg;

	// TODO: Check motor ids.
	bhPidParamsMsg = boardDataToPidParameters(_piData.getLeftBallhandlerBoard().getDataOutput());
	bhPidParamsMsg.motorId = 1;
	_tBhPidParameters.publish(bhPidParamsMsg);

	bhPidParamsMsg = boardDataToPidParameters(_piData.getRightBallhandlerBoard().getDataOutput());
	bhPidParamsMsg.motorId = 2;
	_tBhPidParameters.publish(bhPidParamsMsg);
}

bool cRosAdapterBallhandlers::cbGetAngle(peripheralsInterface::s_get_ballHandlers_angle::Request& request, peripheralsInterface::s_get_ballHandlers_angle::Response& response) {
	TRACE(">");

	response.angle = _piData.getLeftBallhandlerBoard().getDataOutput().ballhandler.angle;

	TRACE("<");

	return true;
}

bool cRosAdapterBallhandlers::cbSetAngle(peripheralsInterface::s_set_ballHandlers_angle::Request& request, peripheralsInterface::s_set_ballHandlers_angle::Response& response) {
	TRACE(">");

	_piData.setBallhandlerAngle(request.angle);

	TRACE("<");

	return true;
}

bool cRosAdapterBallhandlers::cbActivatePassBall(
		peripheralsInterface::s_peripheralsInterface_activatePassBall::Request& request,
		peripheralsInterface::s_peripheralsInterface_activatePassBall::Response& response) {
	passBall passBallParams;
	passBallParams.activate = request.activate;
	passBallParams.bhLeftSpeed = request.bhLeftSpeed;
	passBallParams.bhRightSpeed = request.bhRightSpeed;

	_piData.setPassBallParams(passBallParams);

	return true;
}

bool cRosAdapterBallhandlers::cbEnableBallhandlers(
		peripheralsInterface::s_enable_ballHandlers::Request&,
		peripheralsInterface::s_enable_ballHandlers::Response& response)
{
	BallhandlerSettings settings = _piData.getBallhandlerSettings();
	settings.controlMode = BALLHANDLER_CONTROL_MODE_ON;

	_piData.setBallhandlerSettings(settings);

	return true;
}

bool cRosAdapterBallhandlers::cbDisableBallhandlers(
		peripheralsInterface::s_disable_ballHandlers::Request&,
		peripheralsInterface::s_disable_ballHandlers::Response& response)
{
	BallhandlerSettings settings = _piData.getBallhandlerSettings();
	settings.controlMode = BALLHANDLER_CONTROL_MODE_OFF;

	_piData.setBallhandlerSettings(settings);

	return true;
}

rosMsgs::t_bh_pid_params boardDataToPidParameters(BallhandlerBoardDataOutput boardDataOutput) {
	rosMsgs::t_bh_pid_params bhPidParamsMsg;

	// TODO: Mapping output not correct.
	bhPidParamsMsg.ledYellow = boardDataOutput.motorController.ledYellow;
	bhPidParamsMsg.displacement = 0;
	bhPidParamsMsg.velocity = 0;
	bhPidParamsMsg.angleZero = boardDataOutput.ballhandler.angleZero;
	bhPidParamsMsg.tachoZero = boardDataOutput.ballhandler.tachoZero;
	bhPidParamsMsg.mode = boardDataOutput.motorController.mode;
	bhPidParamsMsg.angle = boardDataOutput.ballhandler.angle;
	bhPidParamsMsg.hasBall = false;

	bhPidParamsMsg.setPoint = 0;
	bhPidParamsMsg.measuredValue = 0;
	bhPidParamsMsg.error = boardDataOutput.motorController.error;
	bhPidParamsMsg.integral = boardDataOutput.motorController.integral;
	bhPidParamsMsg.derivative = 0;
	bhPidParamsMsg.pidOutput = boardDataOutput.motorController.pidOutput;

	bhPidParamsMsg.P = boardDataOutput.motorController.pid.p;
	bhPidParamsMsg.I = boardDataOutput.motorController.pid.i;
	bhPidParamsMsg.D = boardDataOutput.motorController.pid.d;
	bhPidParamsMsg.iTh = boardDataOutput.motorController.pid.iTh;

	bhPidParamsMsg.pwm = boardDataOutput.motorController.pwm;
	bhPidParamsMsg.measureTime = boardDataOutput.motorController.measureTime;
	bhPidParamsMsg.calculationTime = boardDataOutput.motorController.calculationTime;
	bhPidParamsMsg.currentChannelA = boardDataOutput.motorController.currentChannelA;
	bhPidParamsMsg.currentChannelB = boardDataOutput.motorController.currentChannelB;
	bhPidParamsMsg.voltage = boardDataOutput.motorController.voltage;
	bhPidParamsMsg.boardTemperature = boardDataOutput.motorController.boardTemperature;
	bhPidParamsMsg.motorTemperature = 0;

	return bhPidParamsMsg;
}
