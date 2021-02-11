// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * packetConstructorMixedTeam.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#ifndef PACKETCONSTRUCTORMIXEDTEAM_HPP_
#define PACKETCONSTRUCTORMIXEDTEAM_HPP_

#include <vector>

#include "int/types/ballCandidate.hpp"
#include "int/types/obstacleCandidate.hpp"
#include "int/types/robotLocationMixedTeam.hpp"
#include "int/types/packetStructureMixedTeam.hpp"

#include "cByteArray.hpp"

class packetConstructorMixedTeam
{
	public:
		packetConstructorMixedTeam();
		~packetConstructorMixedTeam();

		void setTimeStamp(const uint32_t value);
		void setTeamColor(const uint8_t value);
		void setOriginalTeamId(const uint8_t value);
		void setBallCandidates(std::vector<ballCandidateStructure> candidates);
		void setObstacleCandidates(std::vector<obstacleCandidateStructure> candidates);
		void setRobotLocation(const robotLocationMixedTeamStructure location);
		void setByteArray(Facilities::cByteArray byteArray);

		Facilities::cByteArray getByteArray();
		packetStructureMixedTeam getMixedTeamStructure() const;

	private:
		packetStructureMixedTeam _mtPacket;
};

#endif /* PACKETCONSTRUCTORMIXEDTEAM_HPP_ */
