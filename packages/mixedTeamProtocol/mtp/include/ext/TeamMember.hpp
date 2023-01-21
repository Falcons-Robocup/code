// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_TEAMMEMBER_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_TEAMMEMBER_HPP_

// standard/system headers
#include <string>

// headers from this package
#include "PlayerId.hpp"
#include "Pose.hpp"

namespace mtp
{

struct TeamMember
{
    PlayerId id;
    std::string role;
    std::string intention;
    bool hasBall;
    Pose position;
    Pose velocity;

    TeamMember(PlayerId const &id_) : id(id_) {}
}; // end of class TeamMember

} // end of namespace mtp

#endif
