// Copyright 2017-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

