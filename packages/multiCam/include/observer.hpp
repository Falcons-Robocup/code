// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * observer.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: Tim Kouters
 */

#ifndef OBSERVER_HPP_
#define OBSERVER_HPP_

#include <stddef.h> // size_t
#include <vector>

#include "types/robotLocationType.hpp"
#include "types/ballPositionType.hpp"
#include "types/obstaclePositionType.hpp"
#ifndef NOROS

#include "multiCamStatistics.hpp" // sharedTypes
#endif

class observer
{
    public:
        observer();
        virtual ~observer();

        virtual void update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset);
        virtual void update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset);
        virtual void update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset);
        virtual void update_own_ball_possession(const bool hasPossession);
#ifndef NOROS

        virtual void update_multi_cam_statistics(multiCamStatistics const &multiCamStats);
#endif
        virtual void updateCameraMounting(bool correctlyMounted);
        virtual void updateMinimumLockTime(const float minimumLockTime);

    protected:
        float _cameraOffset;
        float _minimumLockTime;

};

#endif /* COBSERVER_HPP_ */
