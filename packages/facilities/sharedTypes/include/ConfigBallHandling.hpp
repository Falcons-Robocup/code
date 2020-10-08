 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CONFIGBALLHANDLING_HPP_
#define CONFIGBALLHANDLING_HPP_

#include "RtDB2.h" // required for serialization



struct BallHandlingArmConfig
{
    float down = 2500; // sensor value related to down position
    float up = 4000; // sensor value related to up position

    SERIALIZE_DATA(down, up);
};

struct BallHandlingRobotConfig
{
    int robotId = 0;
    BallHandlingArmConfig leftArm;
    BallHandlingArmConfig rightArm;

    SERIALIZE_DATA(robotId, leftArm, rightArm);
};

struct BallPossessionConfig
{
    float angleThresholdOn = 0.4; // angle where ball is seen as detected
    float angleThresholdOff = 0.4; // angle where ball is seen as no longer detected
    float minimumTimeUp = 0.2; // time interval in seconds when ball possession is true after both arms go up

    SERIALIZE_DATA(angleThresholdOn, angleThresholdOff, minimumTimeUp);
};

struct BallHandlingFeedForwardConfig
{
    bool enabledWithoutBall = false;
    bool enabledWithBall = true;
    float factorX = 0.0;
    float factorY = 0.0;
    float factorRz = 0.0;

    SERIALIZE_DATA(enabledWithoutBall, enabledWithBall, factorX, factorY, factorRz);
};

struct BallHandlingExtraPullConfig
{
    // option to generate extra pull force on a lifted ballhandler arm, when the other one is down
    // which can help to bring the ball in
    bool enabledWithoutBall = true;
    bool enabledWithBall = false;
    float setpointVelocity = 300;

    SERIALIZE_DATA(enabledWithoutBall, enabledWithBall, setpointVelocity);
};

struct ConfigBallHandling
{
    // note: angles are fractions, where zero is down (no ball possession) and 1 is up (highest limit)
    // conversion to sensor values is done via arm calibration
    float                         angleSetpoint = 0.8; // control setpoint angle
    float                         armLiftedAngleThreshold = 0.4; // when is arm considered 'lifted'
    BallPossessionConfig          ballPossession;
    BallHandlingFeedForwardConfig feedForward;
    BallHandlingExtraPullConfig   extraPull;
    std::vector<BallHandlingRobotConfig> calibration;

    SERIALIZE_DATA(angleSetpoint, armLiftedAngleThreshold, ballPossession, feedForward, extraPull, calibration);
};

#endif

