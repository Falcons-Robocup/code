// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * observer.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: Tim Kouters
 */

#include "observer.hpp"

#include <math.h>


observer::observer()
{
    _cameraOffset = 0.0;
    _minimumLockTime = 0.0;
}

observer::~observer()
{

}

void observer::update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset)
{

}

void observer::update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset)
{

}
void observer::update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset)
{

}

void observer::update_own_ball_possession(const bool hasPossession)
{

}

void observer::update_multi_cam_statistics(multiCamStatistics const &multiCamStats)
{

}

void observer::updateCameraMounting(bool correctlyMounted)
{
    if(correctlyMounted)
    {
        _cameraOffset = 0.0;
    }
    else
    {
        _cameraOffset = M_PI;
    }
}

void observer::updateMinimumLockTime(const float minimumLockTime)
{
    _minimumLockTime = minimumLockTime;
}

