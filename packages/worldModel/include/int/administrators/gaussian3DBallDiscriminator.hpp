// Copyright 2020-2022 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gaussianBallDiscriminator.hpp
 *
 *  Created on: Nov 26, 2019
 *      Author: lucas
 */

#ifndef GAUSSIAN3DBALLDISCRIMINATOR_HPP_
#define GAUSSIAN3DBALLDISCRIMINATOR_HPP_


#include <vector>

#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "int/administrators/IballDiscriminator.hpp"
#include "diagWorldModel.hpp"
#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"

#include "gaussian3d.hpp"
#include "int/administrators/gaussian3DPosVelObject.hpp"

class Gaussian3DBallDiscriminator : public IballDiscriminator
{
public:
    Gaussian3DBallDiscriminator(WorldModelConfig* wmConfig);
    virtual ~Gaussian3DBallDiscriminator();

    virtual void addMeasurement(const ballMeasurement& measurement);
    virtual void addPossessionMeasurement(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp);
    virtual void performCalculation(rtime timeNow, const Vector2D& pos);

    virtual std::vector<ballClass_t> getBalls() const;
    virtual void getMeasurementsToSync(std::vector<ballMeasurement>& measurementsToSync);
    virtual void fillDiagnostics(diagWorldModel& diagnostics);

private:

    WorldModelConfig* wmConfig;

    std::vector<ballClass_t> balls;

    std::vector<ballMeasurement> measurements;

    // The measurements are maintained in a double buffered manner so that
    // the last cycle measurements are available for diagnostics
    int currentMeasurementBuffer;
    std::vector<Gaussian3DMeasurement> measurementsBuffer[2];
    std::vector<Gaussian3DMeasurement>* gaussianMeasurements;

    std::vector<Gaussian3DPosVelObject> gaussianBalls;

    const double ballMergeThreshold;
    const double ballVelocityMergeThreshold;
    const double measurementMergeThreshold;
    const double minAcceptedConfidence;
    const double blacklistMaxVelocity;
    const double blacklistMaxAcceleration;
    const double flyingMeasurementHeight;
    const double parallelAxisDistanceFactor;
    const double parallelAxisOffset;
    const double parallelAxisElevationFactor;
    const double perpendicularAxisDistanceFactor;
    const double perpendicularAxisOffset;
    const double maxAllowedVariance;
    const double timeFactor;

    double getMeasurementVarianceParallelAxis(const Vector3D& measurementVec, const ballMeasurement& measurement, double elevation);
    double getMeasurementVariancePerpendicularAxis(const Vector3D& measurementVec, const ballMeasurement& measurement);
    Vector3D getMeasurementMean(Vector3D measurementVec, const ballMeasurement& measurement);
    Gaussian3DMeasurement gaussianMeasurementFromBallMeasurement(const ballMeasurement& measurement);
    Gaussian3DMeasurement gaussianMeasurementFromPossesionMeasurement(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp);
    void estimateBallsMovement(rtime const timeNow);
    void removeLostBalls();
    void removeBallsOutsideField();
    void addGaussianMeasurement(const Gaussian3DMeasurement& measurement);
    bool isGaussianMeasurementOfFlyingBall(const Gaussian3DMeasurement& measurement);
    void blacklistFakeBalls();
    void mergeBalls();
    void convertGaussianBallsToOutputBalls();
    double getGaussianBallConfidence(const Gaussian3DPosVelObject& ball);
    void swapMeasurementBuffer();
    std::vector<Gaussian3DMeasurement>* getLastMeasurements();
};

#endif /* GAUSSIANBALLDISCRIMINATOR_HPP_ */
