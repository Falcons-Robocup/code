// Copyright 2016-2019 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballAdministratorMock.hpp
 *
 *  Created on: Nov 25, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLADMINISTRATORMOCH_HPP_
#define BALLADMINISTRATORMOCH_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "int/administrators/ballAdministrator.hpp"

class ballAdministratorMock : public ballAdministrator
{
    public:
    	MOCK_METHOD1(appendBallMeasurements, void(const T_BALL_CANDIDATES measurements));
    	MOCK_METHOD1(overruleBall, void(const ballClass_t ball));
    	MOCK_METHOD1(getLocalBallMeasurements, void(T_BALL_CANDIDATES &measurements));
    	MOCK_METHOD2(performCalculation, void(rtime const timeNow, Vector2D const &pos));
    	MOCK_METHOD1(getBalls, void(std::vector<ballClass_t> &balls));

    private:
    	MOCK_METHOD1(cleanUpTimedOutBallMeasurements, void(rtime const timeNow));
};

#endif /* BALLADMINISTRATORMOCH_HPP_ */
