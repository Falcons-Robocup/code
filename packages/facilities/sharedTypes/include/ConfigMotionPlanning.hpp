 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CONFIGMOTIONPLANNING_HPP_
#define CONFIGMOTIONPLANNING_HPP_

#include "RtDB2.h" // required for serialization



struct GetBallConfig
{
    float obstacleThreshold = 3.0;
    float ballSpeedThreshold = 0.35;
    float ballSpeedScaling = 0.6;

    SERIALIZE_DATA(obstacleThreshold, ballSpeedThreshold, ballSpeedScaling);
};

struct InterceptBallConfig
{
    float obstacleThreshold = 3.0;
    float ballSpeedThreshold = 0.35;
    float ballSpeedScaling = 0.6;
    float captureRadius = 2.1;
    bool activeIntercept = false;

    SERIALIZE_DATA(obstacleThreshold, ballSpeedThreshold, ballSpeedScaling, captureRadius, activeIntercept);
};

struct KeeperMoveConfig
{
    float yMaxOffset = 0.3; // distance "out of the goal"
    float xGoalpostOffset = 0.45; // distance from the goalpost (sideways/X direction)

    SERIALIZE_DATA(yMaxOffset, xGoalpostOffset);
};

struct PassToTargetConfig
{
    float aimSettleTime = 0.2;
    float accuracy = 0.05; // keep trying, optimize for short pass
    float timeout = 1.0;
    float coarseAngle = 0.5;
    float disableBhDelay = 0.1; // for (short) passes, if tuned agressively, ballHandling will cause either small bump or reduce pass power
    float sleepAfterShoot = 0.3;

    SERIALIZE_DATA(aimSettleTime, accuracy, timeout, coarseAngle, disableBhDelay, sleepAfterShoot);
};

struct ShootAtTargetConfig
{
    float aimSettleTime = 0.1;
    float accuracy = 0.2;
    float timeout = 0.6;
    float coarseAngle = 0.5;
    float disableBhDelay = 0.0; 
    float sleepAfterShoot = 0.3;

    SERIALIZE_DATA(aimSettleTime, accuracy, timeout, coarseAngle, disableBhDelay, sleepAfterShoot);
};


struct ConfigMotionPlanning
{
    GetBallConfig           getBallConfig;
    InterceptBallConfig     interceptBallConfig;
    KeeperMoveConfig        keeperMoveConfig;
    PassToTargetConfig      passToTargetConfig;
    ShootAtTargetConfig     shootAtTargetConfig;

    SERIALIZE_DATA(getBallConfig, interceptBallConfig, keeperMoveConfig, passToTargetConfig, shootAtTargetConfig);
};

#endif

