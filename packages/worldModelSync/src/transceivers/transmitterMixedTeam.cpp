// Copyright 2017-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * transmitterMixedTeam.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#include "int/transceivers/transmitterMixedTeam.hpp"

#include "int/configurators/configuratorMixedTeamPacket.hpp"

#include <iostream>
#include <unistd.h> // sleep
#include "cDiagnostics.hpp"
#include "tracing.hpp"

transmitterMixedTeam::transmitterMixedTeam()
{
	_udpTransmitter = NULL;
}

transmitterMixedTeam::~transmitterMixedTeam()
{
	TRACE(">");
	cleanUpTransmitter();
	TRACE("<");
}

void transmitterMixedTeam::reconnect()
{
    TRACE("reconnect");
	try
	{
        TRACE("cleanUpTransmitter");
		cleanUpTransmitter();

        TRACE("construct");
		_udpTransmitter = new Facilities::Network::cTransmitterUDP(
				configuratorMixedTeamPacket::getInstance().getMulticastAddress(),
				configuratorMixedTeamPacket::getInstance().getPortNumber(),
				configuratorMixedTeamPacket::getInstance().getLoopIsEnabled(),
				configuratorMixedTeamPacket::getInstance().getMaxHops());
		TRACE("construct succeeded");

		if(!_udpTransmitter->isSocketOpen())
		{
            TRACE("recurse");
			reconnect();
		}

		TRACE("reconnected");
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void transmitterMixedTeam::sendPacket(Facilities::cByteArray wmPacket)
{
	try
	{
		if(_udpTransmitter->isSocketOpen())
		{
			if(configuratorMixedTeamPacket::getInstance().getMixedTeamIsEnabled())
			{
				_udpTransmitter->sendPacket(&wmPacket);
			}
		}
		else
		{
			TRACE_ERROR("UDP transmitter has no open socket... Reconnecting");
			reconnect();
		}

	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void transmitterMixedTeam::cleanUpTransmitter()
{
	try
	{
		if(_udpTransmitter != NULL)
		{
			delete _udpTransmitter;
			_udpTransmitter = NULL;

			/*
			 * We need to sleep in order to avoid hang ups due to Linux socket
			 */
			TRACE("sleep");
			sleep(20.0);
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
