// Copyright 2016 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * test_main.cpp
 *
 *  Created on: Sep 20, 2016
 *      Author: Edwin Schreuder
 */

#include <int/Ballhandlers.hpp>
#include <int/Motion.hpp>
#include <chrono>
#include <thread>

#include "int/PeripheralsInterfaceData.hpp"
#include "int/PeripheralsInterfaceTypes.hpp"

#include "int/motors/MotionBoard.hpp"

#include "int/DeviceManager.hpp"

using namespace std;

int main(int argc, char **argv) {

	PeripheralsInterfaceData _piData;
	DeviceManager deviceManager(_piData);

	deviceManager.start();

	MotionBoard rearMotionBoard = MotionBoard(MOTION_BOARD_REAR, deviceManager);

	float setpoint = 100;
	if (argc > 1) {
		setpoint = atof(argv[1]);
	}

	while(true) {
		this_thread::sleep_for(chrono::milliseconds(100));

		if (rearMotionBoard.isConnected()) {
			rearMotionBoard.getBoardData();
			rearMotionBoard.setSetpoint(setpoint);
		}
		else {
			// Rear MotionController is not available
//			cout << "Rear Motion Board not available: " << endl;
		}
	}
}
