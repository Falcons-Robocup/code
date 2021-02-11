// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RtdbMatchSettingsAdapter.h
 *
 *  Created on: Oct 27, 2018
 *      Author: robocup
 */

#ifndef INCLUDE_INT_RTDBREFBOXCONFIGADAPTER_H_
#define INCLUDE_INT_RTDBREFBOXCONFIGADAPTER_H_

#include <int/RefboxConfigAdapter.h>
#include <string>

#include "cDbConnection.hpp"


class RtdbRefboxConfigAdapter : public RefboxConfigAdapter
{
public:
    RtdbRefboxConfigAdapter();
    ~RtdbRefboxConfigAdapter();

    void setTeamColor(TeamColor teamColor);
    void setPlayingField(PlayingField playingField);
    void setTTAConfiguration(TTAConfiguration ttaConfig);

private:
    cDbConnection connection;

    template <typename T> void setValue(int database, std::string key, T value);
    template <typename T> T getValue(int database, std::string key);
};

#endif /* INCLUDE_INT_RTDBREFBOXCONFIGADAPTER_H_ */
