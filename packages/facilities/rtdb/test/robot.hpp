// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_RTDB_TEST_ROBOT_HPP__
#define _INCLUDED_RTDB_TEST_ROBOT_HPP__


struct RobotPose
{
    float x = 0.0;
    float y = 0.0;
    float Rz = 0.0;
    SERIALIZE_DATA_FIXED(x, y, Rz);
    // using the _FIXED variant omits the key prefixing, for better compression
    // just try without and see how it looks in rtop/rdump
};

struct Robot
{
    RobotPose pos;
    bool alive = false;
    std::string intention;
    SERIALIZE_DATA(pos, alive, intention);
};


// TODO: it would be nice if this could be generated by RTDB templates? some generic convert to json?
#include <iostream>
inline std::ostream &operator<<(std::ostream &ostr, Robot const &robot)
{
    return ostr << "alive = " << robot.alive << "\n"
        << "pos = (" << robot.pos.x << ", " << robot.pos.y << ", " << robot.pos.Rz << ")\n"
        << "intention = " << robot.intention << "\n";
}

#endif

