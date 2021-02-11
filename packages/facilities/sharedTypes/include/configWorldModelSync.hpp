// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGWORLDMODELSYNC_HPP_
#define CONFIGWORLDMODELSYNC_HPP_

#include "teamIdType.hpp"

#include "RtDB2.h" // required for serialization


struct configWorldModelSync
{
    teamIdType   teamId;
    bool         shareVision;
    std::string  useVisionFrom;
    bool         forceInplay; // TODO: is not really a 'sync' but a more general wm parameter

    SERIALIZE_DATA(teamId, shareVision, useVisionFrom, forceInplay);
};

#endif

