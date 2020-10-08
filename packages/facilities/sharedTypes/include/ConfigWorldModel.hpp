 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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


    SERIALIZE_DATA(objectFit, useOwnHighVision, useFriendlyHighVision, confidenceOmniPref, confidenceNumCams, confidenceMeasLim, confidenceFreshLim, confidenceAgeLim, confidenceFitLim1, confidenceFitLim2, confidenceVLim1, confidenceVLim2, confidenceZLim1, confidenceZLim2, blackListThresholdZ, blackListFloatingDuration, blackListGroundDuration, blackListDefault, solverTrackerConeTolerance, solverTrackerTimeout, solverTrackerBuffer, shareHighVision, confidenceGoodLimit, numberOfBallsWarningThreshold, confidenceMaybeLimit, maxMaybeBalls, friendlyMeasurementsDistance);
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

struct ConfigWorldModel
{
    ConfigWorldModelAdministration administration;
    ConfigWorldModelBallTracker ballTracker;
    ConfigWorldModelLocalization localization;
    ConfigWorldModelObstacleTracker obstacleTracker;

    SERIALIZE_DATA(administration, ballTracker, localization, obstacleTracker);
};

#endif

