// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * receiverWorldModel.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/transceivers/receiverWorldModel.hpp"

#include <iostream>

#include "int/configurators/configuratorWorldModelPacket.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

receiverWorldModel::receiverWorldModel()
{
	_udpReceiver = NULL;
}

receiverWorldModel::~receiverWorldModel()
{
	TRACE("<");
	cleanUpReceiver();
	TRACE(">");
}

void receiverWorldModel::reconnect()
{
    TRACE("reconnect");
	try
	{
        TRACE("cleanUpReceiver");
		cleanUpReceiver();

        TRACE("construct");
		_udpReceiver = new Facilities::Network::cReceiverUDP(
				configuratorWorldModelPacket::getInstance().getMulticastAddress(),
				configuratorWorldModelPacket::getInstance().getPortNumber());
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

void receiverWorldModel::notifyNewPacket(Facilities::cByteArray &data)
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

void receiverWorldModel::cleanUpReceiver()
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
