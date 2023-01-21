// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_OBJECT_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_OBJECT_HPP_

namespace mtp
{

struct Object
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float vz = 0.0;
    float confidence = 0.0;
}; // end of class Object

} // end of namespace mtp

#endif
