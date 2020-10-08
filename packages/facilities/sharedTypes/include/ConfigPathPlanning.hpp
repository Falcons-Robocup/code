 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CONFIGPATHPLANNING_HPP_
#define CONFIGPATHPLANNING_HPP_

#include "RtDB2.h" // required for serialization
#include "CoordinateSystemEnum.hpp"

struct MotionLimitsCommonConfig
{
    float maxDecX        = 4.0;
    float maxDecY        = 4.0;
    float maxDecRz       = 4.0;
    float toleranceXY    = 0.01;
    float toleranceRz    = 0.005;
    float accThresholdX  = 0.0;
    float accThresholdY  = 0.0;
    float accThresholdRz = 0.0;
    SERIALIZE_DATA(maxDecX, maxDecY, maxDecRz, toleranceXY, toleranceRz, accThresholdX, accThresholdY, accThresholdRz);
};

struct MotionLimitsSpecificConfig
{
    float maxVelX         = 1.0;
    float maxVelYforward  = 1.0;
    float maxVelYbackward = 1.0;
    float maxVelRz        = 1.0;
    float maxAccX         = 4.0;
    float maxAccYforward  = 4.0;
    float maxAccYbackward = 4.0;
    float maxAccRz        = 4.0;
    SERIALIZE_DATA(maxVelX, maxVelYforward, maxVelYbackward, maxVelRz, maxAccX, maxAccYforward, maxAccYbackward, maxAccRz);
};

struct MotionLimitsConfig // as used in YAML
{
    bool                          enabled = true;
    MotionLimitsSpecificConfig    withoutBall;
    MotionLimitsSpecificConfig    withBall;
    MotionLimitsCommonConfig      common;
    SERIALIZE_DATA(enabled, withoutBall, withBall, common);
};

struct MotionLimitsConfigPP : public MotionLimitsSpecificConfig, public MotionLimitsCommonConfig
// flattened, as used in PathPlanning after selecting from YAML
{
    bool                          enabled = true;
    SERIALIZE_DATA(enabled, maxVelX, maxVelYforward, maxVelYbackward, maxVelRz, maxAccX, maxAccYforward, maxAccYbackward, maxAccRz, maxDecX, maxDecY, maxDecRz, toleranceXY, toleranceRz, accThresholdX, accThresholdY, accThresholdRz);
};

struct MotionPIDConfig
{
    float XY_P;
    float XY_I;
    float XY_D;
    float RZ_P;
    float RZ_I;
    float RZ_D;
    float maxI_XY;
    float maxI_Rz;
    float fadeI_XY;
    float fadeI_Rz;

    SERIALIZE_DATA(XY_P, XY_I, XY_D, RZ_P, RZ_I, RZ_D, maxI_XY, maxI_Rz, fadeI_XY, fadeI_Rz);
};

enum class VelocitySetpointControllerTypeEnum
{
    NONE,
    LINEAR,
    PID,
    SPG
};

SERIALIZE_ENUM(VelocitySetpointControllerTypeEnum);

struct VelocitySetpointControllerConfig
{
    VelocitySetpointControllerTypeEnum type = VelocitySetpointControllerTypeEnum::NONE;
    CoordinateSystemEnum coordinateSystem = CoordinateSystemEnum::RCS;
    SERIALIZE_DATA(type, coordinateSystem);
};

struct VelocitySetpointControllersConfig
{
    float threshold = 1.0;
    VelocitySetpointControllerConfig longStroke;
    VelocitySetpointControllerConfig shortStroke;
    SERIALIZE_DATA(threshold, longStroke, shortStroke);
};

enum class BoundaryOptionEnum
{
    ALLOW,
    STOP_AND_PASS,
    STOP_AND_FAIL,
    CLIP
};

SERIALIZE_ENUM(BoundaryOptionEnum);

struct BoundaryConfig
{
    BoundaryOptionEnum targetInsideForbiddenArea;
    BoundaryOptionEnum targetOutsideField;
    float fieldMarginX = 0.0;
    float fieldMarginY = 0.0;
    BoundaryOptionEnum targetOnOwnHalf;
    BoundaryOptionEnum targetOnOpponentHalf;
    SERIALIZE_DATA(targetInsideForbiddenArea, targetOutsideField, fieldMarginX, fieldMarginY, targetOnOwnHalf, targetOnOpponentHalf);
};

struct ObstacleAvoidanceConfig
{
    bool enabled = true;
    float robotRadius = 0.26;
    float obstacleRadius = 0.26;
    float distanceScalingFactor = 0.0;
    float speedScalingFactor = 1.0;
    float speedLowerThreshold = 0.3;
    float speedUpperThreshold = 4.0;
    float generatedObstacleSpacing = 0.5;
    float ballClearance = 0.5;
    float groupGapDistance = 0.5;
    float subTargetDistance = 0.5;
    float subTargetExtensionFactor = 0.0;
    SERIALIZE_DATA(enabled, robotRadius, obstacleRadius, distanceScalingFactor, speedScalingFactor, speedLowerThreshold, speedUpperThreshold, ballClearance, generatedObstacleSpacing, groupGapDistance, subTargetDistance, subTargetExtensionFactor);
};

struct SpgConfig
{
    float weightFactorClosedLoop = 0.0;
    float latencyOffset = 0.0;
    bool convergenceWorkaround = false;
    bool synchronizeRotation = true;
    SERIALIZE_DATA(weightFactorClosedLoop, latencyOffset, convergenceWorkaround, synchronizeRotation);
};

struct ForwardDrivingConfig
{
    bool enabled = true;
    float minimumDistance = 2.0;
    SERIALIZE_DATA(enabled, minimumDistance);
};

struct ForwardDrivingConfigs
{
    ForwardDrivingConfig withoutBall;
    ForwardDrivingConfig withBall;
    float radiusRobotToBall = 0.25;
    bool applyLimitsToBall = true;
    SERIALIZE_DATA(withoutBall, withBall, radiusRobotToBall, applyLimitsToBall);
};

struct ConfigPathPlanning
{
    float                              nominalFrequency = 20.0;
    int                                numExtraSettlingTicks = 0;
    VelocitySetpointControllersConfig  velocityControllers;
    ObstacleAvoidanceConfig            obstacleAvoidance;
    BoundaryConfig                     boundaries;
    float                              slowFactor = 0.5;
    ForwardDrivingConfigs              forwardDriving;
    MotionLimitsConfig                 limits;
    bool                               deadzone = false;
    SpgConfig                          setPointGenerator;
    MotionPIDConfig                    pid;

    SERIALIZE_DATA(nominalFrequency, numExtraSettlingTicks, velocityControllers, obstacleAvoidance, boundaries, slowFactor, forwardDriving, limits, deadzone, setPointGenerator, pid);
};

#endif

