 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
