// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * packetStructureWorldModel.hpp
 *
 *  Created on: Mar 21, 2015
 *      Author: Tim Kouters
 */

#ifndef PACKETSTRUCTUREWORLDMODEL_HPP_
#define PACKETSTRUCTUREWORLDMODEL_HPP_

#include <stdint.h>

#include "int/types/ballMeasurement.hpp"
#include "int/types/ballPossession.hpp"
#include "int/types/obstacleMeasurement.hpp"
#include "int/types/robotLocation.hpp"

/*
 * We cannot use vectors because of serialization
 */
#define NR_BALL_MEASUREMENTS 4
#define NR_OBSTACLE_MEASUREMENTS 8

typedef struct
{
		uint8_t packetCounter;
		uint8_t packetVersion;
		uint8_t groupID;
		uint8_t robotID;
		ballMeasurement balls[NR_BALL_MEASUREMENTS];
		ballPossessionStructure ballPossession;
		obstacleMeasurement obstacles[NR_OBSTACLE_MEASUREMENTS];
		robotLocationStructure robotPosition;
} packetStructureWorldModel;

#endif /* PACKETSTRUCTUREWORLDMODEL_HPP_ */
