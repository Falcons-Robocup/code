 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

