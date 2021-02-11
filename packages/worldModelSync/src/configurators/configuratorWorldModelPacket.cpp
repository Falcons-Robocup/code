// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configuratorWorldModelPacket.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/configurators/configuratorWorldModelPacket.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"

configuratorWorldModelPacket::configuratorWorldModelPacket()
{
	_multicastAddress = "224.16.32.74";
	_portNumber = 8001;
	// +10 port offset for now to allow teamB next to teamA for parallel development
    // details in #245 and http://timmel.no-ip.biz:8000/wiki/MSLNetworkSetup#SoftwareIPaddressandportusage
    if (getTeamChar() == 'B')
    {
        _portNumber += 10;
    }
	_loopEnabled = false;
	_maxHops = getHops(); // make sure traffic remains local for simulator 
	_groupID = 0;
	TRACE("constructed configuratorWorldModelPacket");
}

configuratorWorldModelPacket::~configuratorWorldModelPacket()
{

}


void configuratorWorldModelPacket::setMulticastAddress(const std::string address)
{
	_multicastAddress = address;
}

void configuratorWorldModelPacket::setPortNumber(const unsigned int number)
{
	_portNumber = number;
}

void configuratorWorldModelPacket::setLoopIsEnabled(const bool isEnabled)
{
	_loopEnabled = isEnabled;
}

void configuratorWorldModelPacket::setMaxHops(const unsigned int nrHops)
{
	_maxHops = nrHops;
}

void configuratorWorldModelPacket::setGroupID(const unsigned int groupID)
{
	_groupID = groupID;
}

std::string configuratorWorldModelPacket::getMulticastAddress() const
{
	return _multicastAddress;
}

unsigned int configuratorWorldModelPacket::getPortNumber() const
{
	return _portNumber;
}

bool configuratorWorldModelPacket::getLoopIsEnabled() const
{
	return _loopEnabled;
}

unsigned int configuratorWorldModelPacket::getMaxHops() const
{
	return _maxHops;
}

unsigned int configuratorWorldModelPacket::getGroupID() const
{
	return _groupID;
}
