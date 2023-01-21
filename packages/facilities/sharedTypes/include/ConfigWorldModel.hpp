// Copyright 2020-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGWORLDMODEL_HPP_
#define CONFIGWORLDMODEL_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigWorldModelAdministration
{
    float teammember_timeout;
    float obstacle_timeout;
    float ball_timeout;
    SERIALIZE_DATA(teammember_timeout, obstacle_timeout, ball_timeout);
};

struct ConfigWorldModelObjectFit
{
    float depthWeight;          // [1] relative depth weights
    bool speedFitOrder;         // [bool] fit with speed - on robot, we always try to fit with speed
    int minVmeas;               // [1] minimum number of measurements required for speed fitting, otherwise fallback to position-only --> low values affects speed vector responsiveness (a bit magic still)
    int measPerOrder;           // [1] required number of FCS measurements per fit order
    float groupingDt;           // [s] grouping time tolerance for triangulation    --> should be a bit more than heartbeat duration
    float outlierNSigma;        // [1] spread threshold for outlier removal
    int outlierMaxIter;         // number of iterations (1 is minimum, no outlier removal)
    float outlierIterFraction;  // max portion to throw out per iteration, to prevent rejecting good data
    SERIALIZE_DATA(depthWeight, speedFitOrder, minVmeas, measPerOrder, groupingDt, outlierNSigma, outlierMaxIter, outlierIterFraction);
};

struct ConfigWorldModelBallTracker
{
    ConfigWorldModelObjectFit objectFit;

    int ballTrackerAlgorithm;           // 0 - legacy algorithm; 1 - gaussian algorithm    

    bool useOwnHighVision;              // [bool] use own HighVision measurements in algorithm  -->  multiCam produces ball data as if omniCam -- 'high' vision is only produced by frontCam (r1,r3,r4)
    bool useFriendlyHighVision;         // [bool] use friendly HighVision measurements in algorithm

    bool confidenceOmniPref;            // [bool] omnivision required for full confidence
    int confidenceNumCams;              // [1] required number of involved cameras for full confidence 
    int confidenceMeasLim;              // [1] required number of good (i.e. non-outlier) time-clustered measurements for full confidence
    float confidenceFreshLim;           // [s] required freshness of tracker for full confidence, see also tracker timeout
    float confidenceAgeLim;             // [s] required age of tracker for full confidence (if tracker starts at t=1, tcurr=4, then age=3)
    float confidenceFitLim1;            // [1] numerical fit residue threshold - we consider values lower than this threshold good
    float confidenceFitLim2;            // [1] numerical fit residue threshold - we consider values higher than this threshold bad
    float confidenceZLim1;              // [m] threshold for fitted z result - we consider values lower than this threshold good
    float confidenceZLim2;              // [m] threshold for fitted z result - we consider values higher than this threshold bad
    float confidenceVLim1;              // [m/s] speed (vx,vy,vz) threshold - we consider values lower than this threshold good
    float confidenceVLim2;              // [m/s] speed (vx,vy,vz) threshold - we consider values higher than this threshold bad

    float blackListThresholdZ;          // [m] height threshold of ball measurement in FCS used in blacklisting
    float blackListFloatingDuration;    // [s] time after which to blacklist a tracker
    float blackListGroundDuration;      // [s] time after which to whitelist a tracker
    bool blackListDefault;              // [bool] accept a new tracker which has not yet been blacklisted    --> True = cautious; False = aggressive / jumpy
    float solverTrackerConeTolerance;   // [rad] measurement grouping criterion
    float solverTrackerTimeout;         // [s] when to discard trackers
    float solverTrackerBuffer;          // [s] when to discard measurements within tracker

    // BallDiscriminator only
    bool shareHighVision;               // [bool] share own Highvision measurements with friends
    float confidenceGoodLimit;          // [1] all balls with a confidence higher than this threshold are accepted
    int numberOfBallsWarningThreshold;  // [1] warn in case more than this amount of balls are identified
    float confidenceMaybeLimit;         // [1] if no good balls are seen, then this limit is used
    int maxMaybeBalls;                  // [1] amount of 'maybe' balls to accept
    float friendlyMeasurementsDistance; // [m]


    SERIALIZE_DATA(objectFit, ballTrackerAlgorithm, useOwnHighVision, useFriendlyHighVision, confidenceOmniPref, confidenceNumCams, confidenceMeasLim, confidenceFreshLim, confidenceAgeLim, confidenceFitLim1, confidenceFitLim2, confidenceVLim1, confidenceVLim2, confidenceZLim1, confidenceZLim2, blackListThresholdZ, blackListFloatingDuration, blackListGroundDuration, blackListDefault, solverTrackerConeTolerance, solverTrackerTimeout, solverTrackerBuffer, shareHighVision, confidenceGoodLimit, numberOfBallsWarningThreshold, confidenceMaybeLimit, maxMaybeBalls, friendlyMeasurementsDistance);
};

struct ConfigWorldModelLocalization
{
    float errorRatioRadianToMeter;
    float visionOwnWeightFactor;                // sensitive to vision data rate
    float trackerScoreAcceptanceThreshold;
    float trackerTimeout;
    float scoreActivityScale;
    float scoreAgeScale;
    float scoreFreshScale;
    float settlingTime;
    float minimumConfidence;
    float speedLimitXY;
    float speedLimitPhi;
    int visionStabilityLength;
    SERIALIZE_DATA(errorRatioRadianToMeter, visionOwnWeightFactor, trackerScoreAcceptanceThreshold, trackerTimeout, scoreActivityScale, scoreAgeScale, scoreFreshScale, settlingTime, minimumConfidence, speedLimitXY, speedLimitPhi, visionStabilityLength);
};

struct ConfigWorldModelObstacleTracker
{
    ConfigWorldModelObjectFit objectFit;

    float filterXYmemberTolerance;
    float trackerXYTolerance;       // [m] measurement grouping criterion
    float trackerTimeout;           // [s] when to discard measurements
    float extrapolationTimeout;     // [s] when to stop extrapolating
    float speedMinSize;             // rounding threshold: slower -> speed zero
    float speedMaxSize;             // faster is probably glitch -> generate warning
    float speedResidualThreshold;   // in case fit residual is smaller, then we can trust the speed vector, otherwise we choose zero-order fit (this may be quite a sensitive tuning parameter)
    SERIALIZE_DATA(objectFit, filterXYmemberTolerance, trackerXYTolerance, trackerTimeout, extrapolationTimeout, speedMinSize, speedMaxSize, speedResidualThreshold);
};

struct ConfigWorldModelGaussianObstacles
{
    float obstacleMergeThreshold;
    float minAcceptedConfidence;
    float parallelAxisDistanceFactor;
    float parallelAxisOffset;
    float perpendicularAxisDistanceFactor;
    float perpendicularAxisOffset;
    float maxAllowedVariance;

    SERIALIZE_DATA(obstacleMergeThreshold, minAcceptedConfidence, parallelAxisDistanceFactor, parallelAxisOffset, perpendicularAxisDistanceFactor, perpendicularAxisOffset, maxAllowedVariance);
};

struct ConfigWorldModelGaussianBalls
{
    float ballMergeThreshold;
    float minAcceptedConfidence;
    float parallelAxisDistanceFactor;
    float parallelAxisOffset;
    float perpendicularAxisDistanceFactor;
    float perpendicularAxisOffset;
    float maxAllowedVariance;

    SERIALIZE_DATA(ballMergeThreshold, minAcceptedConfidence, parallelAxisDistanceFactor, parallelAxisOffset, perpendicularAxisDistanceFactor, perpendicularAxisOffset, maxAllowedVariance);
};

struct ConfigWorldModelGaussian3DBalls
{
    float ballMergeThreshold;            
    float ballVelocityMergeThreshold;    
    float measurementMergeThreshold;     
    float minAcceptedConfidence;         
    float blacklistMaxVelocity;          
    float blacklistMaxAcceleration;      
    float flyingMeasurementHeight;       
    float parallelAxisDistanceFactor;    
    float parallelAxisOffset;     
    float parallelAxisElevationFactor;
    float perpendicularAxisDistanceFactor;
    float perpendicularAxisOffset;       
    float maxAllowedVariance;         
    float timeFactor;   

    SERIALIZE_DATA(ballMergeThreshold, ballVelocityMergeThreshold, measurementMergeThreshold, minAcceptedConfidence, blacklistMaxVelocity, blacklistMaxAcceleration, flyingMeasurementHeight, parallelAxisDistanceFactor, parallelAxisOffset, parallelAxisElevationFactor, perpendicularAxisDistanceFactor, perpendicularAxisOffset, maxAllowedVariance, timeFactor);
};

struct ConfigWorldModel
{
    ConfigWorldModelAdministration administration;
    ConfigWorldModelBallTracker ballTracker;
    ConfigWorldModelLocalization localization;
    ConfigWorldModelObstacleTracker obstacleTracker;
    ConfigWorldModelGaussianObstacles gaussianObstacles;
    ConfigWorldModelGaussianBalls gaussianBalls;
    ConfigWorldModelGaussian3DBalls gaussian3DBalls;

    SERIALIZE_DATA(administration, ballTracker, localization, obstacleTracker, gaussianObstacles, gaussianBalls, gaussian3DBalls);
};

#endif

