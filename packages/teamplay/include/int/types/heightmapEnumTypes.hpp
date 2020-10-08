 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
