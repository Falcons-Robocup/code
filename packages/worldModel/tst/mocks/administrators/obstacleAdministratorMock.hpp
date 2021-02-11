// Copyright 2016-2019 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleAdministratorMock.hpp
 *
 *  Created on: Nov 25, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLEADMINISTRATORMOCK_HPP_
#define OBSTACLEADMINISTRATORMOCK_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "int/administrators/obstacleAdministrator.hpp"

class obstacleAdministratorMock : public obstacleAdministrator
{
    public:
    	MOCK_METHOD1(appendObstacleMeasurements, void(const T_OBSTACLE_CANDIDATES measurements));
    	MOCK_METHOD1(overruleObstacles, void(const std::vector<obstacleClass_t> obstacles));
    	MOCK_METHOD1(getLocalObstacleMeasurements, void(T_OBSTACLE_CANDIDATES &measurements));
    	MOCK_METHOD1(performCalculation, void(rtime const timeNow));
    	MOCK_METHOD1(getObstacles, void(std::vector<obstacleClass_t> &balls));

    private:
    	MOCK_METHOD1(cleanUpTimedOutObstacleMeasurements, void(rtime const timeNow));
};




#endif /* OBSTACLEADMINISTRATORMOCK_HPP_ */
