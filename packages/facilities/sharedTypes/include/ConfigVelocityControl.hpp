// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGVELOCITYCONTROL_HPP_
#define CONFIGVELOCITYCONTROL_HPP_

#include "RtDB2.h" // required for serialization
#include "CoordinateSystemEnum.hpp"

struct MotionLimitsConfig
{
    float maxVelX         = 1.0;
    float maxVelYforward  = 1.0;
    float maxVelYbackward = 1.0;
    float maxVelRz        = 1.0;
    float maxAccX         = 4.0;
    float maxAccYforward  = 4.0;
    float maxAccYbackward = 4.0;
    float maxAccRz        = 4.0;
    float maxDecX        = 4.0;
    float maxDecY        = 4.0;
    float maxDecRz       = 4.0;
    float accThresholdX  = 0.0;
    float accThresholdY  = 0.0;
    float accThresholdRz = 0.0;
    SERIALIZE_DATA(maxVelX, maxVelYforward, maxVelYbackward, maxVelRz, maxAccX, maxAccYforward, maxAccYbackward, maxAccRz, maxDecX, maxDecY, maxDecRz, accThresholdX, accThresholdY, accThresholdRz);
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

struct SpgConfig
{
    float weightFactorClosedLoop = 0.0;
    float latencyOffset = 0.0;
    bool convergenceWorkaround = false;
    bool synchronizeRotation = true;
    SERIALIZE_DATA(weightFactorClosedLoop, latencyOffset, convergenceWorkaround, synchronizeRotation);
};

struct MotionTypeConfig
{
    VelocitySetpointControllersConfig  velocityControllers;
    MotionLimitsConfig                 limits;
    SpgConfig                          setPointGenerator;
    MotionPIDConfig                    pid;
    SERIALIZE_DATA(velocityControllers, limits, setPointGenerator, pid);
};

struct ConfigVelocityControl
{
    float                                           nominalFrequency = 20.0;
    std::map<std::string, MotionTypeConfig>         motionTypes; // maps to motionTypeEnum -> a new configuration per motionType

    SERIALIZE_DATA(nominalFrequency, motionTypes);
};

#endif

