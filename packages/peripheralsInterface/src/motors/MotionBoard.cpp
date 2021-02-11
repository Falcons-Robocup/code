// Copyright 2016-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * MotionBoard.cpp
 *
 *  Created on: May 3, 2016
 *      Author: Edwin Schreuder
 */

#include "int/motors/MotionBoard.hpp"

#include "int/motors/DeviceManager.hpp"

static ApplicationId MotionBoardTypeToApplicationId(MotionBoardType type) {
	ApplicationId id = UNKNOWN;
	switch(type) {
	case MOTION_BOARD_LEFT:
		id = MOTION_MOTOR_LEFT;
		break;
	case MOTION_BOARD_RIGHT:
		id = MOTION_MOTOR_RIGHT;
		break;
	case MOTION_BOARD_REAR:
		id = MOTION_MOTOR_REAR;
	}

	return id;
}

MotionBoard::MotionBoard(MotionBoardType type, DeviceManager &deviceManager) :
			MotorControllerBoard::MotorControllerBoard(MotionBoardTypeToApplicationId(type), deviceManager) {
	motionData = MotionBoardData();
	settings = MotionBoardSettings();
}

void MotionBoard::update() {
	// Update all common motor parameters.
	MotorControllerBoard::update();
}

void MotionBoard::configure() {
	MotorControllerBoard::configure();

	changeMode(MODE_PID_ENCODER);
	requestCommand(CMD_GET_MODE);
	expectedResponses.push_back(RESP_MODE);

	changePwmLimit(settings.maxPwmValue);
	requestCommand(CMD_GET_PWM_LIMIT);
	expectedResponses.push_back(RESP_PWM_LIMIT);

	changePwmDelta(settings.maxPwmStepValue);
	requestCommand(CMD_GET_PWM_DELTA);
	expectedResponses.push_back(RESP_PWM_DELTA);

	changePidPrimaryProperties(settings.pid.p, settings.pid.i, settings.pid.d, settings.pid.iTh, settings.pid.iMax);
	requestCommand(CMD_GET_PID_PRIMARY_PROPERTIES);
	expectedResponses.push_back(RESP_PID_PRIMARY_PROPERTIES);
}

bool MotionBoard::isConfigurationDone() {
	return true;
}

void MotionBoard::handleDefaultResponse(ReceivePackage &package) {
	MotorControllerBoard::handleDefaultResponse(package);

	motionData.velocity = package.getData<int16_t>(2); // velocity;
	motionData.velocityError = package.getData<int16_t>(3); // velocity error
	motionData.displacementEncTicks = package.getData<int32_t>(0);
	motionData.displacementDistance = package.getData<int32_t>(0) * oneWheelTick; //distance in meters
	motionData.measuredValue = package.getData<int16_t>(2); // TODO:check if it is correct value
	motionData.motorTemperature = ntcResistanceToMotorTemperature(adcVoltageToNtcResistance(adcToAdcVoltage(package.getData<uint16_t>(19))));

	motionData.pidOutput = package.getData<int32_t>(3) / 65536.0; // pidOutput
	motionData.integral = package.getData<int32_t>(2) * motorControllerData.pid.i / 65536.0; // integral * Ki (see xmegamotor/src/pid.c)
	motionData.error = package.getData<int16_t>(3) * ((int32_t)motorControllerData.pid.p << 4) / 65536.0; // error * Kp (see xmegamotor/src/pid.c)
	motionData.derivative = 0; //Implementation still needed
}

MotionBoardDataOutput MotionBoard::getBoardData() {
	update();
	return MotionBoardDataOutput {motorControllerData, motionData};
}

void MotionBoard::setSetpoint(float setpoint) {
	if (configured) {
//		cout << "Setting setpoint to " << setpoint << endl;
		changePrimarySetpoint(setpoint);
	}
}

void MotionBoard::setSettings(MotionBoardSettings _settings) {
	settings = _settings;

	configure();
}
