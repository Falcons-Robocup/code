// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * testIMU.cpp
 *
 *  Created on: July 7, 2018
 *      Author: Jan Feitsma
 */

#include "int/IMU/IMU.hpp"
#include "FalconsRTDB.hpp"

// MAIN
int main(int argc, char **argv)
{
    auto imu = IMU();
    double delta = 0.0;
    double prev = 0.0;
    double t0 = rtime::now();
    
    while (true)
    {
        //Vector3 v = imu.getVector();
        double t = rtime::now();
        float angle = imu.getCompassAngle();
        
        if (angle != prev)
        {
            prev = angle;
            delta = t - t0;
            t0 = t;
        }
        
        //printf("%10.2f %10.2f %10.2f %10.2f %10.2f\n", angle, v.x(), v.y(), v.z(), delta);
        printf("%16.6f %10.2f %10.2f\n", t, angle, delta);
        fflush(stdout);
        
        usleep(10000);
    }
    
    
    return 0;
}

