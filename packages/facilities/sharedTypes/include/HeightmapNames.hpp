// Copyright 2022 Sander de Putter (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapNames.hpp
 *
 *  Created on: Apr 17, 2020
 *      Author: Coen Tempelaars
 */

#ifndef HEIGHTMAPNAMES_HPP_
#define HEIGHTMAPNAMES_HPP_

#include "RtDB2.h" // required for serialization

enum class CompositeHeightmapName
{
    INVALID,
    DEFEND_ATTACKING_OPPONENT,
    DRIBBLE,
    MOVE_TO_FREE_SPOT,
    POSITION_FOR_OPP_SETPIECE,
    POSITION_ATTACKER_FOR_OWN_SETPIECE
};

SERIALIZE_ENUM(CompositeHeightmapName);

#endif /* HEIGHTMAPNAMES_HPP_ */
