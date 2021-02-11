// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBAdapterRobotStatus.hpp
 *
 *  Created on: Oct 30, 2018
 *      Author: Erik Kouters
 */

#ifndef INCLUDE_INT_ADAPTERS_CRTDBADAPTERROBOTSTATUS_HPP_
#define INCLUDE_INT_ADAPTERS_CRTDBADAPTERROBOTSTATUS_HPP_

#include "FalconsRtDB2.hpp"

class cRTDBAdapterRobotStatus
{
public:
    cRTDBAdapterRobotStatus();
	~cRTDBAdapterRobotStatus();

	void initialize();
	void setRobotStatus(bool inPlay);

private:
    RtDB2 *_rtdb;
    int _myRobotId;
};

#endif /* INCLUDE_INT_ADAPTERS_CRTDBADAPTERROBOTSTATUS_HPP_ */
