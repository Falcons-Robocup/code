// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_PLAYERPACKET_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_PLAYERPACKET_HPP_

// standard/system headers
#include <vector>

// headers from this package
#include "Pose.hpp"

// other MSL packages
#include "RtDB2.h" // for serialization

namespace mtp
{

struct Ball
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float vz = 0.0;
    float confidence = 0.0;

    SERIALIZE_DATA(x, y, z, vx, vy, vz, confidence);
}; // end of struct Ball

struct Obstacle
{
    float x = 0.0;
    float y = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float confidence = 0.0;

    SERIALIZE_DATA(x, y, vx, vy, confidence);
}; // end of struct Obstacle

struct PosVel
{
    float x = 0.0;
    float y = 0.0;
    float rz = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float vrz = 0.0;
    float confidence = 0.0;

    PosVel(float x_ = 0.0, float y_ = 0.0, float rz_ = 0.0, float vx_ = 0.0, float vy_ = 0.0, float vrz_ = 0.0, float confidence_ = 0.0)
    {
        x = x_;
        y = y_;
        rz = rz_;
        vx = vx_;
        vy = vy_;
        vrz = vrz_;
        confidence = confidence_;
    }
    PosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence_)
    {
        x = position.x;
        y = position.y;
        rz = position.rz;
        vx = velocity.x;
        vy = velocity.y;
        vrz = velocity.rz;
        confidence = confidence_;
    }

    SERIALIZE_DATA(x, y, rz, vx, vy, vrz, confidence);
}; // end of struct PosVel

struct PlayerPacket
{
    uint8_t                vendor_id = 0;
    uint8_t                shirt_id = 0;
    char                   team_id = '?';
    int32_t                timestamp_ms = 0;
    uint8_t                has_ball = 0;
    std::vector<Ball>      balls;
    std::vector<Obstacle>  obstacles;
    std::vector<PosVel>    self_loc; // optional, can either be empty or contain 1 element
    bool                   is_leader = false;
    std::vector<std::pair<int32_t, uint8_t>> role_allocation; // only filled in if leader
    std::vector<std::pair<uint8_t, float>> role_preference; // optionally filled in, can either be empty or contain 1 element
    uint8_t                role = 0; // current role
    uint8_t                intention = 0;
    uint8_t                error = 0;

    SERIALIZE_DATA(vendor_id, shirt_id, team_id, timestamp_ms, has_ball, balls, obstacles, self_loc, is_leader, role_allocation, role_preference, role, intention, error);
}; // end of struct PlayerPacket

} // end of namespace mtp

#endif
