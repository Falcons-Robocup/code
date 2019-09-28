 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorWorldModelPacket.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/configurators/configuratorWorldModelPacket.hpp"

#include "FalconsCommon.h"
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
