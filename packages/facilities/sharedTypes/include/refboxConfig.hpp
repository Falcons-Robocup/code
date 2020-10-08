 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * refboxConfig.hpp
 *
 *  Created on: Oct 20, 2018
 *      Author: Jan Feitsma
 */

#ifndef REFBOXCONFIG_HPP_
#define REFBOXCONFIG_HPP_

#include "RtDB2.h" // required for serialization

enum class refboxConfigTeamColor
{
    CYAN,
    MAGENTA
};

SERIALIZE_ENUM(refboxConfigTeamColor);

enum class refboxConfigField
{
    FIELD_A,
    FIELD_B,
    FIELD_C,
    FIELD_D,
    FIELD_FALCONS,
    FIELD_LOCALHOST
};

SERIALIZE_ENUM(refboxConfigField);

enum class refboxConfigTTAside
{
    NONE,
    FRONT_LEFT,  // +y, -x in FCS
    FRONT_RIGHT, // +y, +x
    BACK_LEFT,   // -y, -x
    BACK_RIGHT   // -y, +x
};

SERIALIZE_ENUM(refboxConfigTTAside);

struct refboxConfig // associated RTDB key: REFBOX_CONFIG
{
    refboxConfigTeamColor  teamColor;
    refboxConfigField      field;
    refboxConfigTTAside    technicalTeamArea = refboxConfigTTAside::NONE;

    SERIALIZE_DATA_FIXED(teamColor, field, technicalTeamArea);
};

#endif

