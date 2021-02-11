// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagWorldModelLocal.hpp
 *
 *  Created on: March, 2019
 *      Author: Jan Feitsma
 *
 * detailed high-density logging, to remain on robot
 * useful for instance when debugging why a ball suddenly disappears
 */

#ifndef DIAGWORLDMODELLOCAL_HPP_
#define DIAGWORLDMODELLOCAL_HPP_

#include "RtDB2.h" // required for serialization

#include "vec2d.hpp"
#include "objectMeasurement.hpp"
#include "ballResult.hpp"

struct diagLocalizationTracker
{
    int id;
    SERIALIZE_DATA(id);
};

struct diagObjectMeasurement
{
    objectMeasurement m;
    bool used = true;
    SERIALIZE_DATA(m, used);
};

struct diagObjectMeasurementCluster
{
    rtime t;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    bool used = true;
    SERIALIZE_DATA_FIXED(t, x, y, z, used);
};

struct diagBallTracker
{
    int id;
    ballResult result;
    std::vector<diagObjectMeasurement> measurements;
    std::vector<diagObjectMeasurementCluster> measurementClusters;
    float ownGoodDataRate = 0.0; // 0.0 = none, 1.0 is 1 sample per heartbeat
    float outliersFraction = 0.0; // between 0.0 and 1.0
    float age = 0.0; // time since first measurement
    float freshness = 0.0; // time since last measurement
    bool obf = false;
    SERIALIZE_DATA(id, result, measurements, measurementClusters, age, freshness, ownGoodDataRate, outliersFraction, obf);
};

struct diagVector2D
{
    double x;
    double y;
    SERIALIZE_DATA(x, y);
};

struct diagMatrix22
{
    double matrix[2][2];
    SERIALIZE_DATA(matrix);
};

struct diagVector3D
{
    double x;
    double y;
    double z;
    SERIALIZE_DATA(x, y, z);
};

struct diagMatrix33
{
    double matrix[3][3];
    SERIALIZE_DATA(matrix);
};

struct diagGaussian2D
{
    diagVector2D mean;
    diagMatrix22 covariance;
    SERIALIZE_DATA(mean, covariance);
};

struct diagGaussian3D
{
    diagVector3D mean;
    diagMatrix33 covariance;
    SERIALIZE_DATA(mean, covariance);
};

struct diagMeasurement
{
    diagGaussian2D gaussian2d;
    uint8_t measurer_id;
    SERIALIZE_DATA(gaussian2d, measurer_id);
};

struct diagMeasurement3D
{
    diagGaussian3D gaussian3d;
    uint8_t measurer_id;
    SERIALIZE_DATA(gaussian3d, measurer_id);
};

struct diagObstacle
{
    diagGaussian2D gaussian2d;
    bool isTeammember;
    uint8_t robot_id; // only valid if it is a team member

    SERIALIZE_DATA(gaussian2d, isTeammember, robot_id);
};

struct diagConfidence
{
    double total_confidence;
    double position_confidence;
    double measurers_confidence;
    double time_condifence;

    SERIALIZE_DATA(total_confidence, position_confidence, measurers_confidence, time_condifence);
};

struct diagBall
{
    diagGaussian3D position;
    diagGaussian3D velocity;
    bool blacklisted;
    bool bestBall;
    diagConfidence confidence;
    
    SERIALIZE_DATA(position, velocity, blacklisted, bestBall, confidence);
};

struct diagGaussianObstacleDiscriminator
{
    std::vector<diagMeasurement> measurements;
    std::vector<diagObstacle> obstacles;

    SERIALIZE_DATA(measurements, obstacles);
};

struct diagGaussianBallDiscriminator
{
    std::vector<diagMeasurement3D> measurements;
    std::vector<diagBall> balls;

    SERIALIZE_DATA(measurements, balls);
};

struct diagObstacleTracker
{
    int id;
    SERIALIZE_DATA(id);
};

struct diagWorldModelLocal
{
    std::vector<diagLocalizationTracker> localization;
    std::vector<diagBallTracker>         balls;
    std::vector<diagObstacleTracker>     obstacles;
    rtime timestamp;
    
    diagGaussianObstacleDiscriminator gaussianObstacleDiscriminatorData;
    diagGaussianBallDiscriminator gaussianBallDiscriminatorData;

    SERIALIZE_DATA(timestamp, localization, balls, obstacles, gaussianObstacleDiscriminatorData, gaussianBallDiscriminatorData);
    // TODO merge obstacles and gaussianObstacleDiscriminatorData (first was intended as placeholder for second)
    // TODO reuse stuff between balls and obstacles
};

#endif

