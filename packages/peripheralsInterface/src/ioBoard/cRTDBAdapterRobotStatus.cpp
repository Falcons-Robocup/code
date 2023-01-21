// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBAdapterRobotStatus.cpp
 *
 *  Created on: Oct 30, 2018
 *      Author: Erik Kouters
 */

#include "int/ioBoard/cRTDBAdapterRobotStatus.hpp"

#include <iostream>
#include <string>

#include <cDiagnostics.hpp>
#include <falconsCommon.hpp>
#include "tracing.hpp"

using std::cerr;
using std::endl;
using std::string;

cRTDBAdapterRobotStatus::cRTDBAdapterRobotStatus() {
}

cRTDBAdapterRobotStatus::~cRTDBAdapterRobotStatus() {
}

void cRTDBAdapterRobotStatus::initialize() {
    TRACE_FUNCTION("");

    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
}

void cRTDBAdapterRobotStatus::setRobotStatus(bool inPlay) {
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK robotStatus;

	if (inPlay) {
		robotStatus = robotStatusEnum::INPLAY;
	}
	else {
		robotStatus = robotStatusEnum::OUTOFPLAY;
	}

	_rtdb->put(INPLAY_FEEDBACK, &robotStatus);
}
