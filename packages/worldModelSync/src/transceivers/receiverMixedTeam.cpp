// Copyright 2017-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * receiverMixedTeam.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#include "int/transceivers/receiverMixedTeam.hpp"

#include <iostream>

#include "int/configurators/configuratorMixedTeamPacket.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

receiverMixedTeam::receiverMixedTeam()
{
	_udpReceiver = NULL;
}

receiverMixedTeam::~receiverMixedTeam()
{
	TRACE("<");
	cleanUpReceiver();
	TRACE(">");
}

void receiverMixedTeam::reconnect()
{
    TRACE("reconnect");
	try
	{
        TRACE("cleanUpReceiver");
		cleanUpReceiver();

        TRACE("construct");
		_udpReceiver = new Facilities::Network::cReceiverUDP(
				configuratorMixedTeamPacket::getInstance().getMulticastAddress(),
				configuratorMixedTeamPacket::getInstance().getPortNumber());
		_udpReceiver->attachObserver(this);

		if(!_udpReceiver->isSocketOpen())
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

void receiverMixedTeam::notifyNewPacket(Facilities::cByteArray &data)
{
	try
	{
		_fncNotifyNewPacket(data);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void receiverMixedTeam::cleanUpReceiver()
{
	try
	{
		if(_udpReceiver != NULL)
		{
			_udpReceiver->detachObserver(this);
			delete _udpReceiver;
			_udpReceiver = NULL;

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
