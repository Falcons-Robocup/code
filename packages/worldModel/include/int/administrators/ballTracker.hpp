// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballTracker.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLTRACKER_HPP_
#define BALLTRACKER_HPP_

#include <vector>
#include <string>

#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/types/object/blackListType.hpp"
#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/algorithms/objectTracking.hpp"

#define MIN_BALL_HEIGHT 0.0
#define MIN_WARNING_CONFIDENCE 0.0
#define MIN_WARNING_HEIGHT -0.1

struct confidenceDetails
{
    int numCams;
    bool withOmni;
    float camScore;
    float ageScore;
    float measScore;
    float freshScore;
    float zScore;
    float vScore;
    float boundScore;
    float fitScore;
    float fitQuality;
};

class ballTracker
{
    public:
    	ballTracker(const objectMeasurementCache &measurement, const WorldModelConfig* wmConfig);
    	~ballTracker();

    	void addBallMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded);
    	void calculateBall(rtime const timeNow, bool ownBallsFirst = false);
    	bool isTimedOut(rtime const timeNow);
    	ballClass_t getBall() const;
    	bool isBlackListed() const;
    	std::string toStr(rtime const timeNow, bool details = false);
    	std::string xyzDetailsStr();
    	confidenceDetails getDetails() const { return _confDetails; }
        bool getOwnBallsFirst() const { return _ownBallsFirst; }
    	size_t numMeasurements() const { return _ballMeasurements.size(); }

        static void reset();
        void traceMeasurements() const;
        std::vector<objectMeasurementCache> getBallMeasurements() { return _ballMeasurements; }
        void makeDiagnostics(diagBallTracker &diag, rtime const &timeNow);
        
    private:
    	std::vector<objectMeasurementCache> _ballMeasurements;
    	ballClass_t _lastBallResult;
    	confidenceDetails _confDetails;
    	objectTracker _tracker;
    	size_t _trackerID;
    	static size_t _staticTrackerID;
        blackListType _blackList;
        bool _ownBallsFirst;
        int _ownRobotId;
        rtime _lastGroundTimeStamp;
        rtime _lastAirTimeStamp;
    	rtime _t0;
        rtime _tmin;
        rtime _tmax;

        const WorldModelConfig* _wmConfig;

    	void selectOwnOmniMeasurements(std::vector<objectMeasurementCache> &measurements);
        void selectLowMeasurements(std::vector<objectMeasurementCache> &measurements);
        void calculateConfidence(rtime const t);
    	void setBlackList(blackListType b);
        void ensureUniqueMeasurements();
    	void cleanUpTimedOutBallMeasurements(rtime const t);

        bool checkDeltaAngles(const objectMeasurementCache &measurement, Vector3D const &expectedPos);
};

#endif /* BALLTRACKER_HPP_ */
