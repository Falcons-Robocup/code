// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGPATHPLANNING_HPP_
#define CONFIGPATHPLANNING_HPP_

#include "RtDB2.h" // required for serialization

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

struct DeadzoneConfig
{
    bool enabled;
    float toleranceXY    = 0.01;
    float toleranceRz    = 0.005;
    SERIALIZE_DATA(enabled, toleranceXY, toleranceRz);
};

struct TokyoDriftConfig
{
    float toleranceRz    = 1.0; // if deltaPos.Rz > tokyoDrift.toleranceRz -> do tokyo drift; otherwise do normal rotation
    SERIALIZE_DATA(toleranceRz);
};

struct ConfigPathPlanning
{
    int                                numExtraSettlingTicks = 0;
    ObstacleAvoidanceConfig            obstacleAvoidance;
    BoundaryConfig                     boundaries;
    float                              slowFactor = 0.5;
    ForwardDrivingConfigs              forwardDriving;
    DeadzoneConfig                     deadzone;
    TokyoDriftConfig                   tokyoDrift;

    SERIALIZE_DATA(numExtraSettlingTicks, obstacleAvoidance, boundaries, slowFactor, forwardDriving, deadzone, tokyoDrift);
};

#endif

