// Copyright 2015-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cBallPossessionTypes.hpp
 *
 *  Created on: Sep 18, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CBALLPOSSESSIONTYPES_HPP_
#define CBALLPOSSESSIONTYPES_HPP_

#include <stdint.h>
#include "vector2d.hpp"
#include "int/types/cRobotLocationTypes.hpp"

enum class ballPossessionEnum
{
    INVALID = 0,
    FIELD,
    TEAMMEMBER
};

typedef struct
{
	ballPossessionEnum possessionType;
	robotNumber robotID;
	Point2D ballClaimedLocation;
} ballPossession_struct_t;


#endif /* CBALLPOSSESSIONTYPES_HPP_ */
