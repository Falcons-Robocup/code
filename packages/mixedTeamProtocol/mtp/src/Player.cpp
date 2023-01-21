// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/Player.hpp"

// standard/system headers
// ...

using namespace mtp;

Player::Player(PlayerId const &id_)
:
    id(id_)
{
}

Player::~Player()
{
}

void Player::update(PlayerPacket const &p)
{
    packet = p;
}
