// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * localizationTracker.hpp
 *
 *  Created on: Jun 10, 2017
 *      Author: Jan Feitsma
 */

#ifndef LOCALIZATIONTRACKER_HPP_
#define LOCALIZATIONTRACKER_HPP_


#include <deque>
#include "position2d.hpp"

#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotMeasurementType.hpp"

class localizationTracker
{
public:
    localizationTracker(const WorldModelConfig* wmConfig);
    ~localizationTracker();
    
    // setters
    void updateExpectedPosition(Position2D const &deltaDisplacementPos);
    void feedMeasurement(robotMeasurementClass_t const &measurement, double timestampNow);
    void setId(int id);
    void matchPosition(Position2D const &referencePosition);
    
    // getters
    float getCameraMeasurementScore(robotMeasurementClass_t const &measurement) const;
    float getTrackerScore(double timestampNow); // tracker with highest score wins
    float getLocAge(double timestampNow);
    double getLastVisionTimestamp() const;
    double getLastPokeTimestamp() const;
    Position2D getPosition() const;
    float getVisionConfidence() const;
    bool isTimedOut(double timestampNow) const;
    
    // unique ID
    int getId();
    static int requestId();

private:
    // helpers
    void cleanup(double timestampNow);
    float getPositionScore(Position2D const &positionA, Position2D const &positionB) const;
    Position2D mirror(Position2D const &position) const;
    
private:
    // data members
    int _id;
    Position2D _position;
    float _trackerTimeout;
    float _visionConfidence;
    double _creationTimestamp;
    double _lastVisionTimestamp;
    double _lastPokeTimestamp;
    std::deque<double> _recentTimestamps;

    const WorldModelConfig* _wmConfig;
    
};

#endif /* LOCALIZATIONTRACKER_HPP_ */

