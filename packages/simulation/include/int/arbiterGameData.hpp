// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * arbiterGameData.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#ifndef ARBITERGAMEDATA_HPP_
#define ARBITERGAMEDATA_HPP_

#include "gameData.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

class ArbiterGameData : public GameData {
public:
    ArbiterGameData();

    void refresh(const GameData&);

    bool ballIsInNegativeGoal() const;
    bool ballIsInPositiveGoal() const;
    bool ballIsAcrossNegativeGoalline() const;
    bool ballIsAcrossPositiveGoalline() const;
    bool ballIsAcrossSideLine() const;
    bool isScrum() const;

    /* the following attributes are deliberately public */
    TeamID teamLastHoldingBall;    // there is always a team that last touched the ball
    RobotID robotLastHoldingBall;  // there is always a robot that last touched the ball

private:
    typedef boost::accumulators::accumulator_set<float, boost::accumulators::stats<boost::accumulators::tag::rolling_mean > > MovingAverageAccumulator;
    MovingAverageAccumulator ballSpeedAccumulator;
    MovingAverageAccumulator ballToRobotDistanceAccumulatorTeamA;
    MovingAverageAccumulator ballToRobotDistanceAccumulatorTeamB;
};

#endif /* ARBITERGAMEDATA_HPP_ */
