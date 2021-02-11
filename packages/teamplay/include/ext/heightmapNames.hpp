// Copyright 2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapNames.hpp
 *
 *  Created on: Apr 17, 2020
 *      Author: Coen Tempelaars
 */

#ifndef HEIGHTMAPNAMES_HPP_
#define HEIGHTMAPNAMES_HPP_

enum class CompositeHeightmapName
{
    INVALID,
    DEFEND_ATTACKING_OPPONENT,
    DRIBBLE,
    MOVE_TO_FREE_SPOT,
    POSITION_FOR_OPP_SETPIECE,
    POSITION_FOR_OWN_SETPIECE
};

inline char const *enum2str(CompositeHeightmapName const &s)
{
    char const *result = "UNKNOWN";
    switch (s)
    {
        case CompositeHeightmapName::INVALID:
            result = "INVALID";
            break;
        case CompositeHeightmapName::DEFEND_ATTACKING_OPPONENT:
            result = "DEFEND_ATTACKING_OPPONENT";
            break;
        case CompositeHeightmapName::DRIBBLE:
            result = "DRIBBLE";
            break;
        case CompositeHeightmapName::MOVE_TO_FREE_SPOT:
            result = "MOVE_TO_FREE_SPOT";
            break;
        case CompositeHeightmapName::POSITION_FOR_OPP_SETPIECE:
            result = "POSITION_FOR_OPP_SETPIECE";
            break;
        case CompositeHeightmapName::POSITION_FOR_OWN_SETPIECE:
            result = "POSITION_FOR_OWN_SETPIECE";
            break;

        default:
            result = "UNKNOWN";
    }
    return result;
}

#endif /* HEIGHTMAPNAMES_HPP_ */
