// Copyright 2016-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * MotorControllerBoard.hpp
 *
 *  Created on: May 3, 2016
 *      Author: robocup
 */

#ifndef INCLUDE_INT_MOTORS_MOTORCONTROLLERBOARD_HPP_
#define INCLUDE_INT_MOTORS_MOTORCONTROLLERBOARD_HPP_

#include <chrono>
#include <vector>

#include "int/motors/PeripheralsInterfaceData.hpp"
#include "int/motors/Communication.hpp"
#include "int/motors/DeviceManager.hpp"

using namespace std;

class MotorControllerBoard {
public:
	MotorControllerBoard(ApplicationId id, DeviceManager &deviceManager);
	virtual ~MotorControllerBoard();

	bool isConnected();
	bool isConfigured();

protected:
	ApplicationId applicationId;
	string boardName;
	DeviceManager &deviceManager;

	bool configured;
	size_t updateCounter;

	chrono::steady_clock::time_point lastUpdateTime;
	vector<ResponseList> expectedResponses;

	MotorControllerBoardData motorControllerData;

	virtual void update();
	virtual void forceConfiguration();
	virtual void configure();
	virtual bool isConfigurationDone();

	void send(TransmitPackage package);

	void handleCommError(unsigned short commError);
	void handleEncError(unsigned short encError);
	void handlePidError(unsigned short pidError);
	void handlePwmError(unsigned short pwmError);
	void handleSafetyError(unsigned short safetyError);
	void handleSchedulerError(unsigned short schedulerError);
	void handleDrv8301Error(unsigned short drv8301Error);

	// Data retrieve functions.
	void requestCommand(CommandList command);

	// Data send functions.
	void clearErrors(ReceivePackage &package);
	void clearAllErrors();
	void loopback();

	virtual void handleResponse(ReceivePackage &package);
	virtual void handleDefaultResponse(ReceivePackage &package);
	virtual void handleErrorResponse(ReceivePackage &package);
	virtual void handleGainResponse(ReceivePackage &package);
	virtual void handleLedGreenResponse(ReceivePackage &package);
	virtual void handleLedYellowResponse(ReceivePackage &package);
	virtual void handleLoopbackResponse(ReceivePackage &package);
	virtual void handleMotorTimeoutResponse(ReceivePackage &package);
	virtual void handlePidPrimaryPropertiesResponse(ReceivePackage &package);
	virtual void handlePwmLimitResponse(ReceivePackage &package);
	virtual void handlePrimarySetpointResponse(ReceivePackage &package);
	virtual void handlePwmManualResponse(ReceivePackage &package);
	virtual void handleDrv8301Response(ReceivePackage &package);
	virtual void handlePwmDeltaResponse(ReceivePackage &package);
	virtual void handleModeResponse(ReceivePackage &package);
	virtual void handleAngleTachoZeroResponse(ReceivePackage &package);
	virtual void handlePidAnglePropertiesResponse(ReceivePackage &package);
	virtual void handleAngleSetpointResponse(ReceivePackage &package);
	virtual void handleAngleDirectionResponse(ReceivePackage &package);

	void changeGain(unsigned char value);
	void changeLedGreen(bool flag);
	void changeLedYellow(bool flag);
	void changeMotorTimeout(unsigned short value);
	void changePidPrimaryProperties(unsigned short p, unsigned short i, unsigned short d, unsigned short iTh, unsigned long iMax);
	void changePwmManual(short value);
	void changePwmLimit(unsigned short value);
	void changePrimarySetpoint(short value);
	void changeDrv8301();
	void changePwmDelta(unsigned long value);
	void changeMode(ModeList Mode);
	void changePidAngleProperties(unsigned short p, unsigned short i, unsigned short d, unsigned short iTh, unsigned long iMax);
	void changeAngleSetpoint(short value);
	void changeAngleDirection(bool flag);

	static float adcToAdcVoltage(float adc);
	static float adcVoltageToPowerSupply(float adcVoltage);
	static float adcVoltageToNtcResistance(float adcVoltage);
	static float ntcResistanceToBoardTemperature(float ntcResistance);
	static float ntcResistanceToMotorTemperature(float ntcResistance);
};

#endif /* INCLUDE_INT_MOTORS_MOTORCONTROLLERBOARD_HPP_ */
