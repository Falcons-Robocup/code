// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef MOTION_HPP
#define MOTION_HPP

#include "int/PeripheralsInterfaceTypes.hpp"

#include "int/motors/PeripheralsInterfaceData.hpp"
#include "int/motors/DeviceManager.hpp"
#include "int/motors/MotionBoard.hpp"

using namespace std;

class MotionMotor {
public:
	float setpoint;
	MotionBoardDataOutput data;
	bool online;
	string name;
};

class Motion {

public:
	Motion(PeripheralsInterfaceData &piData, MotionBoard& leftMotionBoard, MotionBoard& rightMotionBoard, MotionBoard& rearMotionBoard);
	~Motion();

	void update();

private:
	void watchDog();

	void updateStatus();
	void updateBoardData();
	void updateBoardSetpoints();

	void calculateEncodersInput();
	void calculateVelocityAndDisplacementOutput();

	void traceData();

	PeripheralsInterfaceData &_piData;

	size_t _watchDogCycles;

	MotionBoard& _leftMotionBoard;
	MotionBoard& _rightMotionBoard;
	MotionBoard& _rearMotionBoard;

	MotionMotor _leftMotor;
	MotionMotor _rightMotor;
	MotionMotor _rearMotor;
};
#endif /* CPERIPHERALSINTERFACEMOTION_HPP */
