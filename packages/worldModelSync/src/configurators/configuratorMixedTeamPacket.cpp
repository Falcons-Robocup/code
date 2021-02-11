// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configuratorMixedTeamPacket.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/configurators/configuratorMixedTeamPacket.hpp"

configuratorMixedTeamPacket::configuratorMixedTeamPacket()
{
	_multicastAddress = "224.16.32.75";
	_portNumber = 2005;
	_loopEnabled = false;
	_maxHops = 2;
	_groupID = 0;
	_mixedIsEnabled = false;
}

configuratorMixedTeamPacket::~configuratorMixedTeamPacket()
{

}


void configuratorMixedTeamPacket::setMulticastAddress(const std::string address)
{
	_multicastAddress = address;
}

void configuratorMixedTeamPacket::setPortNumber(const unsigned int number)
{
	_portNumber = number;
}

void configuratorMixedTeamPacket::setLoopIsEnabled(const bool isEnabled)
{
	_loopEnabled = isEnabled;
}

void configuratorMixedTeamPacket::setMaxHops(const unsigned int nrHops)
{
	_maxHops = nrHops;
}

void configuratorMixedTeamPacket::setGroupID(const unsigned int groupID)
{
	_groupID = groupID;
}

void configuratorMixedTeamPacket::setMixedTeamEnabled(const bool isEnabled)
{
	_mixedIsEnabled = isEnabled;
}

std::string configuratorMixedTeamPacket::getMulticastAddress() const
{
	return _multicastAddress;
}

unsigned int configuratorMixedTeamPacket::getPortNumber() const
{
	return _portNumber;
}

bool configuratorMixedTeamPacket::getLoopIsEnabled() const
{
	return _loopEnabled;
}

unsigned int configuratorMixedTeamPacket::getMaxHops() const
{
	return _maxHops;
}

unsigned int configuratorMixedTeamPacket::getGroupID() const
{
	return _groupID;
}

bool configuratorMixedTeamPacket::getMixedTeamIsEnabled() const
{
	return _mixedIsEnabled;
}
