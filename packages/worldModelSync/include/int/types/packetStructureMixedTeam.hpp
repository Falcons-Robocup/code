// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * packetStructureMixedTeam.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#ifndef PACKETSTRUCTUREMIXEDTEAM_HPP_
#define PACKETSTRUCTUREMIXEDTEAM_HPP_

#include <stdint.h>

#include "int/types/ballCandidate.hpp"
#include "int/types/obstacleCandidate.hpp"
#include "int/types/robotLocationMixedTeam.hpp"

#define NR_MIXED_BALL_CANDIDATES 3
#define NR_MIXED_OBSTACLE_CANDIDATES 12

typedef struct
{
	uint8_t  mixedTeamFlag;
	uint8_t  packetVersion;
	uint32_t timestamp;
	uint8_t  teamColor;
	uint8_t  originalTeamId;
	uint8_t  mixedTeamId;
	ballCandidateStructure ballCandicates[NR_MIXED_BALL_CANDIDATES];
	obstacleCandidateStructure obstacleCandidates[NR_MIXED_OBSTACLE_CANDIDATES];
	robotLocationMixedTeamStructure selfPosition;
} packetStructureMixedTeam;


#endif /* PACKETSTRUCTUREMIXEDTEAM_HPP_ */
