// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BallHandlers.hpp
 *
 *  Created on: Mar 1, 2016
 *      Author: Prabhu Mani
 */

#ifndef BALLHANDLERS_HPP_
#define BALLHANDLERS_HPP_

#include "int/motors/PeripheralsInterfaceData.hpp"
#include "int/motors/DeviceManager.hpp"
#include "int/motors/BallhandlerBoard.hpp"

class BallhandlerMotor {
public:
	BallhandlerBoardDataOutput data;
	int setpoint;
	bool online;
};

class Ballhandlers {

public:
	Ballhandlers(PeripheralsInterfaceData& piData, BallhandlerBoard& leftBallhandlerBoard, BallhandlerBoard& rightBallhandlerBoard);
	~Ballhandlers();

	void update();

private:
	void updateBoardSettings();
	void updateControlMode();
	void updateBoardData();
	void updateBoardSetpoints();
	void updateBoardStatus();

	void traceData();

	PeripheralsInterfaceData &_piData;

	BallhandlerBoard& _leftBallhandlerBoard;
	BallhandlerBoard& _rightBallhandlerBoard;

	BallhandlerMotor _leftBallhandler;
	BallhandlerMotor _rightBallhandler;
	
	bool _leftEnabled;
	bool _rightEnabled;
};

#endif /* PERIPHERALSINTERFACEBALLHANDLERS_HPP_ */
