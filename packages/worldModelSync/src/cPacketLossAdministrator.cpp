 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPacketLossAdministrator.cpp
 *
 *  Created on: Jul 16, 2015
 *      Author: Tim Kouters
 */

#include "int/cPacketLossAdministrator.hpp"

#include <stdexcept>

#include "FalconsCommon.h"
#include "tracing.hpp"
#include "cDiagnostics.hpp"


using std::runtime_error;
using std::exception;

cPacketLossAdministrator::cPacketLossAdministrator()
{
    _initialized = false;
	for(size_t i = 0; i < _MAX_ROBOTS; i++)
	{
		/*
		 * Set last received packet number to 0
		 */
		_lastReceivedPacketNumber[i] = 0;

		/*
		 * Set array of received packets to true for each robot
		 */
		for(size_t j = 0; j < _PACKET_ARR_SIZE; j++)
		{
			_receivedPackages[i][j] = true;
		}
	}
}

cPacketLossAdministrator::~cPacketLossAdministrator()
{

}

void cPacketLossAdministrator::notifyPacketNumber(uint8_t packetNumber, uint8_t robotID)
{
	try
	{
		if (robotID >= _MAX_ROBOTS)
		{
			TRACE("robotID (%d) bigger than maximum allowed number in administration (%d)",
					robotID, _MAX_ROBOTS);
			ROS_ERROR("robotID (%d) bigger than maximum allowed number in administration (%d)",
					robotID, _MAX_ROBOTS);
			throw runtime_error("RobotID bigger than administration");
		}

	    
		/* Set all in between missed packages to false */
		if (_initialized) // init robustness - first received packet is not per se index zero
		{
		    uint16_t inbetweenPackets = (((int)packetNumber + _PACKET_ARR_SIZE) - _lastReceivedPacketNumber[robotID] - 1) % _PACKET_ARR_SIZE;
    	    //TRACE("packetNumber=%3d  last=%d  robotID=%d  inbetweenPackets=%3d", packetNumber, _lastReceivedPacketNumber[robotID], robotID, inbetweenPackets);

		    for(uint8_t i = 1; i <= inbetweenPackets; i++)
		    {
			    _receivedPackages[robotID][(_lastReceivedPacketNumber[robotID] + i) % _PACKET_ARR_SIZE] = false;
		    }
        }
        
		/* Set received package to true */
		_receivedPackages[robotID][packetNumber % _PACKET_ARR_SIZE] = true;

		/* Update last received package number for next package */
		_lastReceivedPacketNumber[robotID] = packetNumber;
		_initialized = false;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

float cPacketLossAdministrator::getPacketLoss(uint8_t robotID)
{
    // return as percentage (0..100)
	uint8_t lostPackages = 0;

	/* Iterate over the buffer for this robot and counter the number of missed packages */
	for(uint8_t j = 0; j < _PACKET_ARR_SIZE; j++)
	{
		if(_receivedPackages[robotID][j] == false)
		{
			lostPackages++;
		}
	}

	return lostPackages * 100.0 / _PACKET_ARR_SIZE;
}




