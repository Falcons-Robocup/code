// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

