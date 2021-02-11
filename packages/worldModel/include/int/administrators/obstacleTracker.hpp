// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleTracker.hpp
 *
 *  Created on: Oct 5, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLETRACKER_HPP_
#define OBSTACLETRACKER_HPP_

#include <vector>

#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/algorithms/objectTracking.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "int/types/robot/robotType.hpp"


class obstacleTracker
{
public:
    obstacleTracker(const objectMeasurementCache &measurement, const WorldModelConfig& wmConfig);
    ~obstacleTracker();

    void addObstacleMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded);
    void performCalculation(rtime const timeNow);
    bool isTimedOut(rtime const timeNow);
    void checkFake(std::vector<robotClass_t> const &teamMembers);
    bool isFake() const { return _fake; };

    obstacleClass_t getObstacle() const;
    std::string toStr(rtime const tcurr);

private:
    std::vector<objectMeasurementCache> _obstacleMeasurements;
    objectTracker _tracker;
    obstacleClass_t _lastObstacleResult;
    bool _fake;
    static size_t _staticTrackerID;
    size_t _trackerID;

    const WorldModelConfig& _wmConfig;

    void setConfidence(rtime const t);
    void cleanUpTimedOutObstacleMeasurements(rtime const timeNow);
};

#endif /* OBSTACLETRACKER_HPP_ */
