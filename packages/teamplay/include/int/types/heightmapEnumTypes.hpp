// Copyright 2017-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapEnumTypes.hpp
 *
 *  Created on: Dec 14, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HEIGHTMAPENUMTYPES_HPP_
#define HEIGHTMAPENUMTYPES_HPP_

namespace teamplay
{

enum class heightmapEnum
{
    AVOID_BALL = 0,
    AVOID_OBSTACLES,
    AVOID_TEAM_MATES,
    BETWEEN_POI_AND_CLOSEST_OBSTACLE,
    CLOSE_TO_BALL_CLAIMED_LOCATION,
    CLOSE_TO_OWN_POS,
    IN_FRONT_OF_OPP_GOAL,
    NEAR_OBSTACLES,
    NEAR_OWN_GOAL,
    OBS_BLOCKING_BALL,
    OBS_BLOCKING_OPP_GOAL,
    OBS_BLOCKING_TEAMMATES,

    SIZE_OF_ENUM
};

inline char const *enum2str(heightmapEnum const &s)
{
    char const *result = "UNKNOWN";
    switch (s)
    {
        case heightmapEnum::AVOID_BALL:
            result = "AVOID_BALL";
            break;
        case heightmapEnum::AVOID_OBSTACLES:
            result = "AVOID_OBSTACLES";
            break;
        case heightmapEnum::AVOID_TEAM_MATES:
            result = "AVOID_TEAM_MATES";
            break;
        case heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE:
            result = "BETWEEN_POI_AND_CLOSEST_OBSTACLE";
            break;
        case heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION:
            result = "CLOSE_TO_BALL_CLAIMED_LOCATION";
            break;
        case heightmapEnum::CLOSE_TO_OWN_POS:
            result = "CLOSE_TO_OWN_POS";
            break;
        case heightmapEnum::IN_FRONT_OF_OPP_GOAL:
            result = "IN_FRONT_OF_OPP_GOAL";
            break;
        case heightmapEnum::NEAR_OBSTACLES:
            result = "NEAR_OBSTACLES";
            break;
        case heightmapEnum::NEAR_OWN_GOAL:
            result = "NEAR_OWN_GOAL";
            break;
        case heightmapEnum::OBS_BLOCKING_BALL:
            result = "OBS_BLOCKING_BALL";
            break;
        case heightmapEnum::OBS_BLOCKING_OPP_GOAL:
            result = "OBS_BLOCKING_OPP_GOAL";
            break;
        case heightmapEnum::OBS_BLOCKING_TEAMMATES:
            result = "OBS_BLOCKING_TEAMMATES";
            break;
        case heightmapEnum::SIZE_OF_ENUM:
            result = "SIZE_OF_ENUM";
            break;

        default:
            result = "UNKNOWN";
    }
    return result;
}

} /* namespace teamplay */

#endif /* HEIGHTMAPENUMTYPES_HPP_ */
