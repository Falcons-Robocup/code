// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_POSE_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_POSE_HPP_

namespace mtp
{

struct Pose
{
    float x = 0.0;
    float y = 0.0;
    float rz = 0.0;
    Pose(float x_ = 0.0, float y_ = 0.0, float rz_ = 0.0)
    {
        x = x_;
        y = y_;
        rz = rz_;
    }
}; // end of class Pose

} // end of namespace mtp

#endif
