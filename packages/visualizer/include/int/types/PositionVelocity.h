// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PositionVelocity.h
 *
 *  Created on: October 3rd, 2016
 *      Author: Diana Koenraadt
 */

#ifndef POSITIONVELOCITY_H
#define POSITIONVELOCITY_H

#include <math.h>

struct PositionVelocity
{
    PositionVelocity()
    {
        x = 0;
        y = 0;
        z = 0;
        phi = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        vphi = 0;
    }

    PositionVelocity(float p_x, float p_y) 
        : PositionVelocity()
    {
        x = p_x;
        y = p_y;
    }

    PositionVelocity(float p_x, float p_y, float p_z) 
        : PositionVelocity(p_x, p_y)
    {
        z = p_z;
    }

    PositionVelocity(float p_x, float p_y, float p_z, float p_phi) 
        : PositionVelocity(p_x, p_y, p_z)
    {
        phi = p_phi;
    }

    PositionVelocity(float p_x, float p_y, float p_z, float p_vx, float p_vy, float p_vz) 
        : PositionVelocity(p_x, p_y, p_z)
    {
        vx = p_vx;
        vy = p_vy;
        vz = p_vz;
    }

    PositionVelocity(float p_x, float p_y, float p_z, float p_phi, float p_vx, float p_vy, float p_vz) 
        : PositionVelocity(p_x, p_y, p_z, p_phi)
    {
        vx = p_vx;
        vy = p_vy;
        vz = p_vz;
    }

    PositionVelocity(float p_x, float p_y, float p_z, float p_phi, float p_vx, float p_vy, float p_vz, float p_vphi) 
        : PositionVelocity(p_x, p_y, p_z, p_phi, p_vx, p_vy, p_vz)
    {
        vphi = p_vphi;
    }

    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float phi; // in x/y plane, counter-clockwise, phi=0 corresponds to x+ direction
    float vphi;
};

#endif // POSITIONVELOCITY_H
