// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * transmitterWorldModel.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/transceivers/transmitterWorldModel.hpp"

#include "int/configurators/configuratorWorldModelPacket.hpp"

#include <iostream>
#include <unistd.h> // sleep
#include "cDiagnostics.hpp"
#include "tracing.hpp"

transmitterWorldModel::transmitterWorldModel()
{
	_udpTransmitter = NULL;
}

transmitterWorldModel::~transmitterWorldModel()
{
	TRACE(">");
	cleanUpTransmitter();
	TRACE("<");
}

void transmitterWorldModel::reconnect()
{
    TRACE("reconnect");
	try
	{
        TRACE("cleanUpTransmitter");
		cleanUpTransmitter();

        TRACE("construct");
		_udpTransmitter = new Facilities::Network::cTransmitterUDP(
				configuratorWorldModelPacket::getInstance().getMulticastAddress(),
				configuratorWorldModelPacket::getInstance().getPortNumber(),
				configuratorWorldModelPacket::getInstance().getLoopIsEnabled(),
				configuratorWorldModelPacket::getInstance().getMaxHops());
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

void transmitterWorldModel::sendPacket(Facilities::cByteArray wmPacket)
{
	try
	{
		if(_udpTransmitter->isSocketOpen())
		{
			_udpTransmitter->sendPacket(&wmPacket);
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

void transmitterWorldModel::cleanUpTransmitter()
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
