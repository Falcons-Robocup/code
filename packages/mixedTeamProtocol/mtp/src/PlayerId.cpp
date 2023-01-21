// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "ext/PlayerId.hpp"

// standard/system headers
#include <stdexcept>
#include <string>


using namespace mtp;

PlayerId::PlayerId(int v, int s, char t)
:
    vendorId(v),
    shirtId(s),
    teamId(t)
{
    // check validity
    if (vendorId < 1)
    {
        throw std::runtime_error("invalid vendor id: " + std::to_string(vendorId));
    }
    if (shirtId < 1)
    {
        throw std::runtime_error("invalid shirt id: " + std::to_string(shirtId));
    }
    if (teamId == '?')
    {
        throw std::runtime_error("invalid team id: " + std::to_string(teamId));
    }
}

PlayerId::PlayerId(PlayerId const &other)
:
    vendorId(other.vendorId),
    shirtId(other.shirtId),
    teamId(other.teamId)
{
}

int PlayerId::hash() const
{
    // TODO make nicer... need to discuss with Rob - uuid + some diagnostics facility perhaps
    //return 1000 * (teamId == 'B') + 100 * vendorId + shirtId; // no, this does not work due to agent=10 limitation in RTDB
    return shirtId + 5 * (teamId == 'B'); // specifically designed for test suite ... !
}

std::string PlayerId::describe() const
{
    char buf[80] = {0};
    sprintf(buf, "vendor=%-2d shirt=%-2d team=%c hash=%-6d", vendorId, shirtId, teamId, hash());
    return std::string(buf);
}

bool PlayerId::valid() const
{
    // check validity
    return (vendorId > 0) && (shirtId > 0) && (teamId != '?');
}

bool mtp::operator<(PlayerId const &p1, PlayerId const &p2)
{
    return p1.hash() < p2.hash();
}

bool mtp::operator==(PlayerId const &p1, PlayerId const &p2)
{
    return p1.hash() == p2.hash();
}
