// Copyright 2016-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BallhandlerBoard.hpp
 *
 *  Created on: May 3, 2016
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_MOTORS_BALLHANDLERBOARD_HPP_
#define INCLUDE_INT_MOTORS_BALLHANDLERBOARD_HPP_

#include <string>

#include "int/motors/MotorControllerBoard.hpp"

#include "int/motors/Communication.hpp"
#include "int/motors/DeviceManager.hpp"

using namespace std;

enum BallhandlerBoardType {
	BALLHANDLER_BOARD_RIGHT,
	BALLHANDLER_BOARD_LEFT
};

class BallhandlerBoard : public MotorControllerBoard {
public:
	BallhandlerBoard(BallhandlerBoardType type, DeviceManager &deviceManager);

	BallhandlerBoardDataOutput getBoardData();
	void setSetpoint(int angle, int setpoint);
	void setSettings(BallhandlerBoardSettings settings);
	void setControlMode(BallhandlerBoardControlMode controlMode);
    BallhandlerBoardControlMode getControlMode();

protected:
	virtual void configure();
	virtual bool isConfigurationDone();

	virtual void handleAngleTachoZeroResponse(ReceivePackage &package);
	virtual void handleDefaultResponse(ReceivePackage &package);

	void finishCalibration();
	void updateControlMode();

	BallhandlerBoardData ballhandlerData;
	BallhandlerBoardSettings ballhandlerSettings;
	BallhandlerBoardControlMode ballhandlerControlMode;

	bool calibrated;
};

#endif /* INCLUDE_INT_MOTORS_BALLHANDLERBOARD_HPP_ */
