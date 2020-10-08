 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
