 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cInputUdpAdapter.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: Jan Feitsma
 */

#include "int/adapters/cInputUdpAdapter.hpp"

#include "FalconsCommon.h"
#include "tracing.hpp"

using namespace std;

cInputUdpAdapter::cInputUdpAdapter(string multicastAddress, int multicastPort, float burstWindow)
{
    // setup the UDP receiver
    TRACE("constructing receiver");
    _multicastAddress = multicastAddress;
    _multicastPort = multicastPort;
    _pReceiverUDP = NULL;
    _lastTimestamp = boost::posix_time::microsec_clock::local_time();
    _burstWindow = burstWindow;

    reconnect();

    TRACE("cInputUdpAdapter construction done");
}

cInputUdpAdapter::~cInputUdpAdapter()
{
}


// implement the abstract notification callback (defined in cAbstractObserverByteArray)
void cInputUdpAdapter::notifyNewPacket(Facilities::cByteArray &data)
{
    // convert to string
    std::vector<uint8_t> vdata;
    data.getData(vdata);
    string dataString(vdata.begin(), vdata.end());
    
    TRACE("received message: %s", dataString.c_str());
    
    // check message against the last one
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - _lastTimestamp;
    if ((diff.total_milliseconds() < 1e3 * _burstWindow) && (dataString == _lastMsg))
    {
        // early exit: this is a miniburst
        TRACE("miniburst detected");
    }
    else
    {
        // relay the string to cRobotControl
        TRACE("calling cRobotControl");
        cRobotControl::getInstance().parse(dataString);
    }
    
    // store message and timestamp
    _lastMsg = dataString;
    _lastTimestamp = boost::posix_time::microsec_clock::local_time();
}

void cInputUdpAdapter::reconnect()
{
	TRACE("reconnect");
	try
	{
		TRACE("cleanUpSocket");
		cleanUpSocket();

		TRACE("construct");
		_pReceiverUDP = new Facilities::Network::cReceiverUDP(_multicastAddress, _multicastPort);
		TRACE("construct succeeded");

		if(!_pReceiverUDP->isSocketOpen())
		{
			TRACE("recurse");
			reconnect();
		}

		_pReceiverUDP->attachObserver(this);

		TRACE("reconnected");
	}
	catch(std::exception &e)
	{
		TRACE("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void cInputUdpAdapter::cleanUpSocket()
{
	try
	{
		if(_pReceiverUDP != NULL)
		{
			delete _pReceiverUDP;
			_pReceiverUDP = NULL;

			/*
			 * We need to sleep in order to avoid hang ups due to Linux socket
			 */
			TRACE("sleep");
			sleep(20.0);
		}
	}
	catch(std::exception &e)
	{
		TRACE("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

