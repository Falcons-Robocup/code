// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * identifierGenerator.cpp
 *
 *  Created on: Sep 4, 2016
 *      Author: Tim Kouters
 */

#include "int/facilities/identifierGenerator.hpp"

identifierGenerator::identifierGenerator(uint8_t robotID)
{
	_robotID = robotID;
	_uniqueID = 0;
}

identifierGenerator::~identifierGenerator()
/*
 * When Chuck Norris enters a room,
 * he doesn't turn the lights on, he turns the dark off.
 */
{

}

uniqueObjectID identifierGenerator::getUniqueID()
{
	uniqueObjectID uID;

	uID.robotID = _robotID;
	uID.uniqueID = _uniqueID++;

	return uID;
}
