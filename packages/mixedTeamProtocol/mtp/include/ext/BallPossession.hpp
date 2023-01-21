// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_BALL_POSSESSION_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_BALL_POSSESSION_HPP_

namespace mtp
{

enum class BallPossessionEnum
{
    UNDEFINED = 0,
    FIELD,
    SELF,
    TEAM,
    OPPONENT
};

struct BallPossession
{
    BallPossessionEnum type = BallPossessionEnum::UNDEFINED;
    // shirtId is only filled in for TEAM ballpossession
    // TODO: use PlayerId instead? how to label opponents?
    int shirtId = 0;
};

} // end of namespace mtp

#endif
