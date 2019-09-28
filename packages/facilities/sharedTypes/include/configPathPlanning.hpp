 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CONFIGPATHPLANNING_HPP_
#define CONFIGPATHPLANNING_HPP_

#include "RtDB2.h" // required for serialization


struct motionLimitersConfig
{
    float maxVelXY;
    float maxVelXY_withBall;
    float maxVelPhi;
    float maxVelPhi_withBall;
    float maxAccXY;
    float maxAccPhi;
    float tolerationXY;
    float tolerationPhi;
    float relativeSpeedFactorX;
    float relativeSpeedFactorY;
    float relativeSpeedFactorPhi;
    float obstacleAvoidanceScalingFactor;
    float obstacleAvoidanceDistanceFactor;
    
    SERIALIZE_DATA(maxVelXY, maxVelXY_withBall, maxVelPhi, maxVelPhi_withBall, maxAccXY, maxAccPhi, tolerationXY, tolerationPhi, relativeSpeedFactorX, relativeSpeedFactorY, relativeSpeedFactorPhi, obstacleAvoidanceScalingFactor, obstacleAvoidanceDistanceFactor);
};

struct motionPIDConfig
{
    float X_P;
    float X_I;
    float X_D;
    float Y_P;
    float Y_I;
    float Y_D;
    float XY_P;
    float XY_I;
    float XY_D;
    float PHI_P;
    float PHI_I;
    float PHI_D;
    float maxI_XY;
    float maxI_Phi;

    SERIALIZE_DATA(X_P, X_I, X_D, Y_P, Y_I, Y_D, XY_P, XY_I, XY_D, PHI_P, PHI_I, PHI_D, maxI_XY, maxI_Phi);
};

struct motionProfileConfig
{
    motionLimitersConfig limiters;
    motionPIDConfig pid;
    
    float brake_deceleration;
    float brake_gamma;
    float tokyoDrift_radius;
    float tokyoDrift_step_angle;
    float tokyoDrift_facing_target_tol;

    SERIALIZE_DATA(limiters, pid, brake_deceleration, brake_gamma, tokyoDrift_radius, tokyoDrift_step_angle, tokyoDrift_facing_target_tol);
};

struct configPathPlanning
{
    bool obstacleAvoidanceEnabled;

    motionProfileConfig normal;
    motionProfileConfig setpiece;

    SERIALIZE_DATA(obstacleAvoidanceEnabled, normal, setpiece);
};

#endif

