 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
