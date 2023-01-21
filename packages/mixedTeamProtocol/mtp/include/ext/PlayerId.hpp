// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_PLAYERID_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_PLAYERID_HPP_

// standard/system headers
#include <string>

namespace mtp
{

typedef int PlayerIdHash; // TODO: Rob is considering to use uuid's, moving away from traditional int agent id and all associated limitations
// (however we should take care that this doesn't make things more difficult to diagnose (?))
// perhaps this PlayerId class belongs in RTDB? It would also help if would be serializable.

class PlayerId
{
public:
    PlayerId(int v, int s, char t = 'A');
    PlayerId(PlayerId const &other);

    PlayerIdHash hash() const;
    std::string describe() const;
    bool valid() const;

    const int vendorId;
    const int shirtId;
    const char teamId;

}; // end of class PlayerId

bool operator<(PlayerId const &p1, PlayerId const &p2);
bool operator==(PlayerId const &p1, PlayerId const &p2);

} // end of namespace mtp

#endif
