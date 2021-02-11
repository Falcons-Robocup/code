// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballDiscriminator.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLDISCRIMINATOR_HPP_
#define BALLDISCRIMINATOR_HPP_


#include <vector>

#include "int/administrators/IballDiscriminator.hpp"
#include "diagWorldModel.hpp"
#include "ballMeasurement.hpp"
#include "int/administrators/ballTracker.hpp"
#include "int/types/ball/ballType.hpp"

class ballDiscriminator : public IballDiscriminator
{
    public:
    	ballDiscriminator(const WorldModelConfig& wmConfig);
    	virtual ~ballDiscriminator();

    	virtual void addMeasurement(const ballMeasurement& measurement);
    	virtual void performCalculation(rtime timeNow, const Vector2D& pos);

    	virtual std::vector<ballClass_t> getBalls() const;
        virtual void getMeasurementsToSync(std::vector<ballMeasurement>& measurements);
        virtual void fillDiagnostics(diagWorldModel& diagnostics);

    private:
        const float MAX_BALL_HEIGHT = 5.0;
        int _ownRobotId = 0;

    	std::vector<ballTracker> _ballTrackers;
        std::vector<ballClass_t> _balls;

        const WorldModelConfig& _wmConfig;
        
    	void removeTimedOutTrackers(rtime const timeNow);
        void ownBallsFirst(rtime const timeNow, Vector2D const &pos);
        bool ignoreHighVision(const ballMeasurement &measurement);
    	void selectGoodBalls(rtime const timeNow);
    	void traceTrackers(rtime const timeNow, bool all = false);
};

#endif /* BALLDISCRIMINATOR_HPP_ */
