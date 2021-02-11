// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gaussianBallDiscriminator.hpp
 *
 *  Created on: Nov 26, 2019
 *      Author: lucas
 */

#ifndef GAUSSIANBALLDISCRIMINATOR_HPP_
#define GAUSSIANBALLDISCRIMINATOR_HPP_


#include <vector>

#include "int/administrators/IballDiscriminator.hpp"
#include "diagWorldModel.hpp"
#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"

#include "gaussian2d.hpp"

class GaussianBallDiscriminator : public IballDiscriminator
{
public:
    GaussianBallDiscriminator();
    virtual ~GaussianBallDiscriminator();

    virtual void addMeasurement(const ballMeasurement& measurement);
    virtual void performCalculation(rtime timeNow, const Vector2D& pos);

    virtual std::vector<ballClass_t> getBalls() const;
    virtual void getMeasurementsToSync(std::vector<ballMeasurement>& measurementsToSync);
    virtual void fillDiagnostics(diagWorldModel& diagnostics);

private:
    std::vector<ballClass_t> balls;

    std::vector<ballMeasurement> measurements;

    // The measurements are maintained in a double buffered manner so that
    // the last cycle measurements are available for diagnostics
    int currentMeasurementBuffer;
    std::vector<GaussianMeasurement> measurementsBuffer[2];
    std::vector<GaussianMeasurement>* gaussianMeasurements;

    std::vector<GaussianPosVelObject> gaussianBalls;

    double getMeasurementVarianceParallelAxis(const Vector2D& measurementVec, const ballMeasurement& measurement);
    double getMeasurementVariancePerpendicularAxis(const Vector2D& measurementVec, const ballMeasurement& measurement);
    GaussianMeasurement gaussianMeasurementFromBallMeasurement(const ballMeasurement& measurement);
    void estimateBallsMovement(rtime const timeNow);
    void removeLostBalls();
    void removeBallsOutsideField();
    void addGaussianMeasurement(const GaussianMeasurement& measurement);
    void convertGaussianBallsToOutputBalls();
    double getGaussianBallConfidence(const GaussianPosVelObject& ball);
    void swapMeasurementBuffer();
    std::vector<GaussianMeasurement>* getLastMeasurements();
};

#endif /* GAUSSIANBALLDISCRIMINATOR_HPP_ */
