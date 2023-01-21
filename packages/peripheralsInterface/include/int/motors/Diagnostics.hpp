// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Diagnostics.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_DIAGNOSTICS_HPP_
#define INCLUDE_INT_DIAGNOSTICS_HPP_

#include <thread>

#include <cDiagnostics.hpp>
#include "FalconsRTDB.hpp"

#include "int/motors/PeripheralsInterfaceData.hpp"
#include "int/motors/VoltageMonitor.hpp"

class Diagnostics {
public:
	Diagnostics(PeripheralsInterfaceData& piData, bool ballhandlersAvailable);

	void start();
	void stop();

private:
	PeripheralsInterfaceData& piData;
	VoltageMonitor voltageMonitor;

	std::thread diagnosticsThread;
	bool started;
    RtDB2 *_rtdb = NULL;

	size_t previousNumberOfDevices;
	size_t desiredNumberOfDevices;

	size_t numberOfOnlineBoards;
	bool previousLeftMotionBoardOnline;
	bool previousRightMotionBoardOnline;
	bool previousRearMotionBoardOnline;
	bool previousLeftBallhandlerBoard;
	bool previousRightBallhandlerBoard;

	void timer();
	void addSensorData();
	void addStatus();
	void addBoardStatus();

	void logDeviceStatus(size_t numberOfDevices);
	void logBoardStatus(bool previousBoardOnline, bool boardOnline, string name);

	bool updateBoardStatus(bool boardOnline);
};

#endif /* INCLUDE_INT_DIAGNOSTICS_HPP_ */
