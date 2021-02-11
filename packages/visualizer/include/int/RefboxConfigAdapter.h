// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RefboxConfigAdapter.h
 *
 *  Created on: Oct 27, 2018
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_REFBOXCONFIGADAPTER_H_
#define INCLUDE_INT_REFBOXCONFIGADAPTER_H_

/*
* Pure virtual interface for settings and/or accessing match settings.
*/
class RefboxConfigAdapter
{
public:
    virtual ~RefboxConfigAdapter() {};

    enum TeamColor
    {
        CYAN,
        MAGENTA
    };

    enum PlayingField
    {
        FIELD_A,
        FIELD_B,
        FIELD_C,
        FIELD_D,
        FIELD_FALCONS,
        FIELD_LOCALHOST
    };

    enum TTAConfiguration
    {
        NONE,
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    virtual void setTeamColor(RefboxConfigAdapter::TeamColor teamColor) = 0;
    virtual void setPlayingField(RefboxConfigAdapter::PlayingField playingField) = 0;
    virtual void setTTAConfiguration(RefboxConfigAdapter::TTAConfiguration ttaConfig) = 0;
};

#endif /* INCLUDE_INT_REFBOXCONFIGADAPTER_H_ */
