 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorMixedTeamPacket.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/configurators/configuratorMixedTeamPacket.hpp"

configuratorMixedTeamPacket::configuratorMixedTeamPacket()
{
	_multicastAddress = "224.16.32.75";
	_portNumber = 2005;
	_loopEnabled = false;
	_maxHops = 2;
	_groupID = 0;
	_mixedIsEnabled = false;
}

configuratorMixedTeamPacket::~configuratorMixedTeamPacket()
{

}


void configuratorMixedTeamPacket::setMulticastAddress(const std::string address)
{
	_multicastAddress = address;
}

void configuratorMixedTeamPacket::setPortNumber(const unsigned int number)
{
	_portNumber = number;
}

void configuratorMixedTeamPacket::setLoopIsEnabled(const bool isEnabled)
{
	_loopEnabled = isEnabled;
}

void configuratorMixedTeamPacket::setMaxHops(const unsigned int nrHops)
{
	_maxHops = nrHops;
}

void configuratorMixedTeamPacket::setGroupID(const unsigned int groupID)
{
	_groupID = groupID;
}

void configuratorMixedTeamPacket::setMixedTeamEnabled(const bool isEnabled)
{
	_mixedIsEnabled = isEnabled;
}

std::string configuratorMixedTeamPacket::getMulticastAddress() const
{
	return _multicastAddress;
}

unsigned int configuratorMixedTeamPacket::getPortNumber() const
{
	return _portNumber;
}

bool configuratorMixedTeamPacket::getLoopIsEnabled() const
{
	return _loopEnabled;
}

unsigned int configuratorMixedTeamPacket::getMaxHops() const
{
	return _maxHops;
}

unsigned int configuratorMixedTeamPacket::getGroupID() const
{
	return _groupID;
}

bool configuratorMixedTeamPacket::getMixedTeamIsEnabled() const
{
	return _mixedIsEnabled;
}
