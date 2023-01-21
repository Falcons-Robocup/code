// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_PLAYER_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_PLAYER_HPP_

// headers from this package
#include "ext/PlayerId.hpp"
#include "int/PlayerPacket.hpp"

namespace mtp
{

class Player
{
public:
    Player(PlayerId const &id);
    ~Player();

    void update(PlayerPacket const &p);

    const PlayerId id;
    PlayerPacket packet;
}; // end of class Player

} // end of namespace mtp

#endif
