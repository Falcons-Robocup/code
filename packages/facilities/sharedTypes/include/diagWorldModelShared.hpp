// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagWorldModelShared.hpp
 *
 *  Created on: Jul 23, 2018
 *      Author: Jan Feitsma
 *
 * this struct is intended for sharing from each robot to coach
 */

#ifndef DIAGWORLDMODELSHARED_HPP_
#define DIAGWORLDMODELSHARED_HPP_

#include "pose.hpp"
#include <string>

#include "RtDB2.h" // required for serialization


struct diagWorldModelShared
{
    // general
    rtime              timestamp;
    float              duration;
    bool               inplay;
    std::string        teamActivity;
    // localization
    int                numVisionCandidates;
    float              visionLocAge;
    int                numMotorDisplacementSamples;
    pose               bestVisionCandidate;
    float              visionConfidence;
    float              visionNoiseXY;
    float              visionNoisePhi;
    bool               locationValid;
    // ball possession breakdown
    bool               ballPossessionBallHandlers;
    bool               ballPossessionVision;
    // ball tracking (high-level)
    // NOTE: 
    // we cannot log and share low-level ball tracking details, since it is quite a lot of data, heavy on bandwidth
    // however, we can log those details locally on each robot, as well as on coach
    // this is required for development and troubleshooting - see diagBallTracking.hpp
    int                numBallTrackers;
    int                bestTrackerId;
    bool               ownBallsFirst;
    // obstacle tracking
    int                numObstacleTrackers;

    SERIALIZE_DATA(timestamp, duration, inplay, teamActivity, numVisionCandidates, visionLocAge, numMotorDisplacementSamples, bestVisionCandidate, visionConfidence, visionNoiseXY, visionNoisePhi, locationValid, ballPossessionBallHandlers, ballPossessionVision, numBallTrackers, bestTrackerId, ownBallsFirst, numObstacleTrackers);
};

#endif

