// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleDiscriminator.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: lucas
 */

#ifndef OBSTACLEDISCRIMINATOR_HPP_
#define OBSTACLEDISCRIMINATOR_HPP_

#include <vector>

#include "int/administrators/IobstacleDiscriminator.hpp"
#include "int/types/robot/robotType.hpp"
#include "obstacleMeasurement.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "falconsCommon.hpp"

#include "matrix22.hpp"
#include "gaussian2d.hpp"

class obstacleDiscriminator : public IobstacleDiscriminator
{
public:
    obstacleDiscriminator();
    virtual ~obstacleDiscriminator();

    virtual void addMeasurement(const obstacleMeasurement& measurement);
    virtual void performCalculation(rtime const timeNow, const std::vector<robotClass_t>& teamMembers);
    virtual std::vector<obstacleClass_t> getObstacles() const;
    virtual int numTrackers() const;

    virtual void fillDiagnostics(diagWorldModel &diagnostics);

private:
    std::vector<obstacleClass_t> obstacles;

    // The measurements are maintained in a double buffered manner so that
    // the last cycle measurements are available for diagnostics
    int currentMeasurementBuffer;
    std::vector<GaussianMeasurement> measurementsBuffer[2];
    std::vector<GaussianMeasurement>* measurements;

    std::vector<GaussianObstacle> gaussianObstacles;
    const double obstacleMergeThreshold;

    GaussianMeasurement gaussianMeasurementFromObstacleMeasurement(const obstacleMeasurement& measurement);
    GaussianPosition gaussianPositionFromRobotType(const robotClass_t& measurement);
    double getMeasurementVarianceParallelAxis(const Vector2D& measurementVec);
    double getMeasurementVariancePerpendicularAxis(const Vector2D& measurementVec);
    double getGaussianObstacleConfidence(const GaussianObstacle& obstacle);
    std::vector<GaussianObstacle>::iterator addGaussianMeasurement(const GaussianMeasurement& measurement);
    void removeLostObstacles();
    void removeObstaclesOutsideField();
    void estimateObstaclesMovement(rtime const timeNow);
    void updateTeammembers(const std::vector<robotClass_t>& teamMembers);
    void convertGaussianObstaclesToOutputObstacles();
    void swapMeasurementBuffer();
    std::vector<GaussianMeasurement>* getLastMeasurements();
};

#endif /* OBSTACLEDISCRIMINATORGAUSSIAN_HPP_ */
