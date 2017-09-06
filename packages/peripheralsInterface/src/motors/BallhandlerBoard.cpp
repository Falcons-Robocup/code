 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * BallhandlerBoard.cpp
 *
 *  Created on: May 3, 2016
 *      Author: Edwin Schreuder
 */

// TODO: Check if values from board are correct.

#include "int/motors/BallhandlerBoard.hpp"

static ApplicationId BallhandlerBoardTypeToApplicationId(BallhandlerBoardType type) {
	ApplicationId id = UNKNOWN;

	switch(type) {
	case BALLHANDLER_BOARD_LEFT:
		id = BH_MOTOR_LEFT;
		break;
	case BALLHANDLER_BOARD_RIGHT:
		id = BH_MOTOR_RIGHT;
		break;
	}

	return id;
}

BallhandlerBoard::BallhandlerBoard(BallhandlerBoardType type, DeviceManager &deviceManager) :
		MotorControllerBoard::MotorControllerBoard(BallhandlerBoardTypeToApplicationId(type), deviceManager) {

	ballhandlerData = BallhandlerBoardData();
	ballhandlerSettings = BallhandlerBoardSettings();
	ballhandlerControlMode = BALLHANDLER_CONTROL_MODE_ON;

	angleCalibrated = false;
}

void BallhandlerBoard::setSetpoint(float angle, float setpoint) {
	if (configured) {
		// Set angle setpoint.
		changeAngleSetpoint(ballhandlerData.angleZero + (((ballhandlerSettings.maxAngle - ballhandlerData.angleZero) / 100) * angle));

		//TODO: Remove ugly feedforward factor.)
		// Set feedforward data
		changePrimarySetpoint(ballhandlerData.tachoZero + (setpoint * ballhandlerSettings.feedForwardFactor));
	}
}

void BallhandlerBoard::setSettings(BallhandlerBoardSettings settings) {
	ballhandlerSettings = settings;

	configure();
}

void BallhandlerBoard::setControlMode(BallhandlerBoardControlMode controlMode) {

	if (controlMode != ballhandlerControlMode) {
		ballhandlerControlMode = controlMode;

		if (configured) {
			// Prepare for configuration
			forceConfiguration();

			// Change the actual mode
			updateControlMode();
		}
	}
}

BallhandlerBoardDataOutput BallhandlerBoard::getBoardData() {
	update();
	return BallhandlerBoardDataOutput {motorControllerData, ballhandlerData};
}

void BallhandlerBoard::configure() {
	/*
	 * Set the mode for ball handler motors.
	 * This also sets the motor speed to zero (no spinning ballHandler is a setPoint around 2330)
	 */
	MotorControllerBoard::configure();

	changeAngleDirection(false);
	requestCommand(CMD_GET_ANGLE_DIRECTION);
	expectedResponses.push_back(RESP_ANGLE_DIRECTION);

	changeMode(MODE_PID_ANGLE);
	requestCommand(CMD_GET_MODE);
	expectedResponses.push_back(RESP_MODE);

	requestCommand(CMD_GET_ANGLE_TACHO_ZERO);
	expectedResponses.push_back(RESP_ANGLE_TACHO_ZERO);
	angleCalibrated = false;

	changeMode(MODE_PID_ANGLE);
	requestCommand(CMD_GET_MODE);
	expectedResponses.push_back(RESP_MODE);

	changePwmLimit(ballhandlerSettings.maxPwmValue);
	requestCommand(CMD_GET_PWM_LIMIT);
	expectedResponses.push_back(RESP_PWM_LIMIT);

	changePwmDelta(ballhandlerSettings.maxPwmStepValue);
	requestCommand(CMD_GET_PWM_DELTA);
	expectedResponses.push_back(RESP_PWM_DELTA);

	changePidPrimaryProperties(
			ballhandlerSettings.pid.p,
			ballhandlerSettings.pid.i,
			ballhandlerSettings.pid.d,
			ballhandlerSettings.pid.iTh,
			ballhandlerSettings.pid.iMax);
	requestCommand(CMD_GET_PID_PRIMARY_PROPERTIES);
	expectedResponses.push_back(RESP_PID_PRIMARY_PROPERTIES);

	changePidAngleProperties(ballhandlerSettings.anglePid.p,
			ballhandlerSettings.anglePid.i,
			ballhandlerSettings.anglePid.d,
			ballhandlerSettings.anglePid.iTh,
			ballhandlerSettings.anglePid.iMax);
	requestCommand(CMD_GET_PID_ANGLE_PROPERTIES);
	expectedResponses.push_back(RESP_PID_ANGLE_PROPERTIES);
}

bool BallhandlerBoard::isConfigurationDone() {

	// TODO: Ballhandlerboard is configured when angle zero is calibrated.
	return angleCalibrated;
}

void BallhandlerBoard::handleAngleTachoZeroResponse(ReceivePackage &package) {
	cout << "INFO    : [" << boardName << "] angle zero " << package.getData<uint16_t>(0) <<" tacho zero "<< package.getData<uint16_t>(1) << endl;

	ballhandlerData.angleZero = package.getData<uint16_t>(0);
	ballhandlerData.tachoZero = package.getData<uint16_t>(1);

	calibrateAngle();
}

void BallhandlerBoard::handleDefaultResponse(ReceivePackage &package) {
	MotorControllerBoard::handleDefaultResponse(package);

	ballhandlerData.angle = package.getData<uint16_t>(19);
}

void BallhandlerBoard::calibrateAngle() {
	// TODO: This calibration routine needs a better implementation.

	if (ballhandlerData.angleZero != 0)
	{
		angleCalibrated = true;
		updateControlMode();
	} else {
		angleCalibrated = false;
		requestCommand(CMD_GET_ANGLE_TACHO_ZERO);
		expectedResponses.push_back(RESP_ANGLE_TACHO_ZERO);
	}
}

void BallhandlerBoard::updateControlMode() {

	switch (ballhandlerControlMode) {
	case BALLHANDLER_CONTROL_MODE_OFF:
		changeMode(MODE_DISABLE_MOTOR_POWER);
		requestCommand(CMD_GET_MODE);
		expectedResponses.push_back(RESP_MODE);
		break;
	case BALLHANDLER_CONTROL_MODE_ON:
		changeMode(MODE_PID_ANGLE);
		requestCommand(CMD_GET_MODE);
		expectedResponses.push_back(RESP_MODE);
	}
}
