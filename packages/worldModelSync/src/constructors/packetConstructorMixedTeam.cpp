 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * packetConstructorMixedTeam.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#include "int/constructors/packetConstructorMixedTeam.hpp"

#include "int/configurators/configuratorMixedTeamPacket.hpp"
#include "int/types/mixedTeamConversionConstants.hpp"

#include "cDiagnostics.hpp"

packetConstructorMixedTeam::packetConstructorMixedTeam()
{
	_mtPacket.mixedTeamFlag = MIXED_TEAM_IDENTIFIER;
	_mtPacket.packetVersion = DEFAULT_PACKET_VERSION;
	_mtPacket.timestamp = UNUSED_MIXED_PACKET_VALUE;


	for(size_t i = 0; i < NR_MIXED_BALL_CANDIDATES; i++)
    {
        _mtPacket.ballCandicates[i].x  = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.ballCandicates[i].y  = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.ballCandicates[i].z  = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.ballCandicates[i].vx = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.ballCandicates[i].vy = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.ballCandicates[i].vz = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.ballCandicates[i].confidence = 0;
    }

    for(size_t i = 0; i < NR_MIXED_OBSTACLE_CANDIDATES; i++)
    {
        _mtPacket.obstacleCandidates[i].x  = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.obstacleCandidates[i].y  = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.obstacleCandidates[i].vx = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.obstacleCandidates[i].vy = UNUSED_MIXED_PACKET_VALUE;
        _mtPacket.obstacleCandidates[i].confidence = 0;
    }

    _mtPacket.selfPosition.x      = UNUSED_MIXED_PACKET_VALUE;
    _mtPacket.selfPosition.y      = UNUSED_MIXED_PACKET_VALUE;
    _mtPacket.selfPosition.theta  = UNUSED_MIXED_PACKET_VALUE;
    _mtPacket.selfPosition.vx     = UNUSED_MIXED_PACKET_VALUE;
    _mtPacket.selfPosition.vy     = UNUSED_MIXED_PACKET_VALUE;
    _mtPacket.selfPosition.vtheta = UNUSED_MIXED_PACKET_VALUE;
    _mtPacket.selfPosition.confidence = 0;
}

packetConstructorMixedTeam::~packetConstructorMixedTeam()
{

}

void packetConstructorMixedTeam::setTimeStamp(const uint32_t value)
{
	_mtPacket.timestamp = value;
}

void packetConstructorMixedTeam::setTeamColor(const uint8_t value)
{
	_mtPacket.teamColor = value;
}

void packetConstructorMixedTeam::setOriginalTeamId(const uint8_t value)
{
	_mtPacket.originalTeamId = value;
}

void packetConstructorMixedTeam::setBallCandidates(std::vector<ballCandidateStructure> candidates)
{
	for(size_t i = 0; ((i < candidates.size()) && (i < NR_MIXED_BALL_CANDIDATES)); i++)
	{
		_mtPacket.ballCandicates[i] = candidates.at(i);
	}
}

void packetConstructorMixedTeam::setObstacleCandidates(std::vector<obstacleCandidateStructure> candidates)
{
	for(size_t i = 0; ((i < candidates.size()) && (i < NR_MIXED_OBSTACLE_CANDIDATES)); i++)
	{
		_mtPacket.obstacleCandidates[i] = candidates.at(i);
	}
}

void packetConstructorMixedTeam::setRobotLocation(const robotLocationMixedTeamStructure location)
{
	_mtPacket.selfPosition = location;
}

void packetConstructorMixedTeam::setByteArray(Facilities::cByteArray byteArray)
{
	std::vector<uint8_t> array;
	byteArray.getData(array);

	if(array.size() == sizeof(packetStructureMixedTeam))
	{
		packetStructureMixedTeam *pd = reinterpret_cast<packetStructureMixedTeam*>(array.data());
		_mtPacket = (*pd);
	}
	else
	{
		TRACE_ERROR("Received wrong size Mixed Team Package %d, expected %d",
				array.size(), sizeof(packetStructureMixedTeam));
	}
}

Facilities::cByteArray packetConstructorMixedTeam::getByteArray()
{
	std::vector<uint8_t> array;
	Facilities::cByteArray retArray;

	uint8_t *pd = reinterpret_cast<uint8_t *>(&_mtPacket);
    array.insert(array.end(), pd, pd + sizeof(_mtPacket));
    retArray.setData(array);
    return retArray;
}

packetStructureMixedTeam packetConstructorMixedTeam::getMixedTeamStructure() const
{
	return _mtPacket;
}

