 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * intentionSync.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: Tim Kouters
 */

#include "int/types/intentionSync.hpp"

#include <stdexcept>

#include "ports.hpp"     // from facilities/networkUDP
#include "addresses.hpp" // from facilities/networkUDP
#include <FalconsCommon.h>

intentionSync::intentionSync()
{
	try
	{
		_udpReceiver = NULL;
		_udpTransmitter = NULL;
		reconnect();

	} catch (std::exception &e)
	{
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

}

intentionSync::~intentionSync()
{
	try
	{

	} catch (std::exception &e)
	{
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

}

void intentionSync::sendIntention(const intentionStruct intention)
{
	try
	{
		if(_currentConnectionType != GetPrimaryConnectionType())
		{
			reconnect();
		}

		if((_udpTransmitter->isSocketOpen()) && (_udpTransmitter != NULL))
		{
			Facilities::Network::cByteArray byteArr;
			std::vector<uint8_t> array;

			intentionStruct intentionCopy = intention;
			uint8_t *pd = reinterpret_cast<uint8_t *>(&intentionCopy);
			array.insert(array.end(), pd, pd + sizeof(intentionStruct));
			byteArr.setData(array);

			_udpTransmitter->sendPacket(&byteArr);
		}

		/*
		 * Update own administration as well for own robot
		 */
		//if(_fncReceiverIntention)
		//{
		//	_fncReceiverIntention(intention);
		//}
	} catch (std::exception &e)
	{
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

}

void intentionSync::notifyNewPacket(Facilities::Network::cByteArray &data)
{
	intentionStruct receivedPacket;
	std::vector<uint8_t> array;
	data.getData(array);
	intentionStruct *pd = reinterpret_cast<intentionStruct*>(array.data());
	receivedPacket = (*pd);

	if(_fncReceiverIntention)
	{
		_fncReceiverIntention(receivedPacket);
	}
}

void intentionSync::reconnect()
{
	TRACE("reconnect");
	try
	{
		TRACE("cleanUpSocket");

		if((_udpReceiver != NULL) || (_udpTransmitter != NULL))
		{
			cleanUpSocket();
		}

		TRACE("construct");

		std::string multicastAddress = Facilities::Network::getMulticastAddress();
		int port = Facilities::Network::getPort(Facilities::Network::PORT_TEAMPLAY_INTENTION, 0);

		_udpReceiver = new Facilities::Network::cReceiverUDP(multicastAddress, port);
		_udpTransmitter = new Facilities::Network::cTransmitterUDP(multicastAddress, port, true, getHops());
		TRACE("construct succeeded");

		if(!(_udpReceiver->isSocketOpen()) && (_udpTransmitter->isSocketOpen()))
		{
			TRACE("recurse");
			reconnect();
		}
		_currentConnectionType = GetPrimaryConnectionType();
		_udpReceiver->attachObserver(this);
		TRACE("reconnected");
	}
	catch(std::exception &e)
	{
		TRACE("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void intentionSync::cleanUpSocket()
{
	try
	{
		if(_udpReceiver != NULL)
		{
			delete _udpReceiver;
			_udpReceiver = NULL;
		}

		if(_udpTransmitter != NULL)
		{
			delete _udpTransmitter;
			_udpTransmitter = NULL;
		}

		/*
		 * We need to sleep in order to avoid hang ups due to Linux socket
		 */
		TRACE("sleep");
		sleep(20.0);
	}
	catch(std::exception &e)
	{
		TRACE("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

