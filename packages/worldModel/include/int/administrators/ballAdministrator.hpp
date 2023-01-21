// Copyright 2016-2022 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballAdministrator.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLADMINISTRATOR_HPP_
#define BALLADMINISTRATOR_HPP_

#include <stdint.h>
#include <map>
#include <vector>

#include "diagWorldModel.hpp"

#include "int/adapters/configurators/WorldModelConfig.hpp"

#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/administrators/IballDiscriminator.hpp"
#include "uniqueObjectID.hpp"

class ballAdministrator
{
    public:
    	ballAdministrator(WorldModelConfig& wmConfig);
    	virtual ~ballAdministrator();

    	virtual void appendBallMeasurements(const std::vector<ballMeasurement> measurements);
        virtual void appendBallPossessionMeasurements(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp);
    	virtual void overruleBall(const ballClass_t ball);
    	virtual void getLocalBallMeasurements(std::vector<ballMeasurement> &measurements);
    	virtual void performCalculation(rtime const timeNow, Vector2D const &pos);

    	virtual void getBalls(std::vector<ballClass_t> &balls);
        void fillDiagnostics(diagWorldModel &diagnostics);

    private:
    	uint8_t _ownRobotID;
    	std::map<uint8_t, ballClass_t> _overruledBalls;
    	std::map<uniqueObjectID, ballMeasurement> _ballMeasurements;

        WorldModelConfig& _wmConfig;

    	IballDiscriminator* _ballDiscriminator;

    	virtual void cleanUpTimedOutBallMeasurements(rtime const timeNow);
};

#endif /* BALLADMINISTRATOR_HPP_ */
