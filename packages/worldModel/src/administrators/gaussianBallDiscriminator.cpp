// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gaussianBallDiscriminator.cpp
 *
 *  Created on: Nov 26, 2019
 *      Author: lucas
 */

#include "int/administrators/gaussianBallDiscriminator.hpp"

#include "falconsCommon.hpp"

#include "cEnvironmentField.hpp"
#include "linalgcv.hpp"


const static double MIN_ACCEPTED_CONFIDENCE = 0.4;
const static double BALL_MERGE_THRESHOLD = 0.01;


GaussianBallDiscriminator::GaussianBallDiscriminator() :
    currentMeasurementBuffer(0),
    gaussianMeasurements(&(measurementsBuffer[currentMeasurementBuffer]))
{

}

GaussianBallDiscriminator::~GaussianBallDiscriminator()
{

}

double GaussianBallDiscriminator::getMeasurementVarianceParallelAxis(const Vector2D& measurementVec, const ballMeasurement& measurement)
{
    double distanceFactor = 0.30;
    double size = measurementVec.size();
    double offset = 0.01;

    return offset + size*distanceFactor;
}

double GaussianBallDiscriminator::getMeasurementVariancePerpendicularAxis(const Vector2D& measurementVec, const ballMeasurement& measurement)
{
    //int ownRobotId = getRobotNumber();

    double distanceFactor = 0.02;
    double size = measurementVec.size();
    double offset = 0.01;

    // if(measurement.identifier.robotID == ownRobotId)
    // {
    //     distanceFactor = 0.002;
    // }

    return offset + size*distanceFactor;
}

GaussianMeasurement GaussianBallDiscriminator::gaussianMeasurementFromBallMeasurement(const ballMeasurement& measurement)
{
    Vector3D positionFcs = object2fcs(  measurement.cameraX,
                                        measurement.cameraY,
                                        measurement.cameraZ,
                                        measurement.cameraPhi,
                                        measurement.azimuth,
                                        measurement.elevation,
                                        measurement.radius);

    Vector2D ballPos(positionFcs.x, positionFcs.y);
    Vector2D cameraPos(measurement.cameraX, measurement.cameraY);

    Vector2D measurementVec = ballPos - cameraPos;
    Vector2D measurementOrthogonal(-measurementVec.y, measurementVec.x);
    double varianceParallel = getMeasurementVarianceParallelAxis(measurementVec, measurement);
    double varianceOrthogal = getMeasurementVariancePerpendicularAxis(measurementVec, measurement);

    Gaussian2D ballGaussianPos(ballPos, measurementVec, varianceParallel, measurementOrthogonal, varianceOrthogal);

    //printf("V1:[%f,%f] V2:[%f,%f] var1:%f var2:%f\n",measurementVec.x, measurementVec.y, measurementOrthogonal.x,measurementOrthogonal.y, varianceParallel, varianceOrthogal);
    // Vector2D mean = ballGaussianPos.getMean();
    // Matrix22 covariance = ballGaussianPos.getCovariance();
    // printf("M:([%f,%f],[[%f,%f],[%f,%f]]),\n", mean.x, mean.y, covariance.matrix[0][0], covariance.matrix[0][1], covariance.matrix[1][0], covariance.matrix[1][1]);

    GaussianPosition positionGaussian(ballGaussianPos, measurement.timestamp);
    GaussianMeasurement gaussianMeasurement(positionGaussian, measurement.identifier.robotID);
    return gaussianMeasurement;
}

void GaussianBallDiscriminator::addMeasurement(const ballMeasurement& measurement)
{    
    GaussianMeasurement gaussianMeasurement = gaussianMeasurementFromBallMeasurement(measurement);

    measurements.push_back(measurement);
    gaussianMeasurements->push_back(gaussianMeasurement);
}

void GaussianBallDiscriminator::estimateBallsMovement(rtime const timeNow)
{
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        double dt = double(timeNow - it->timestamp);
        it->estimateMovement(dt);
    }
}

void GaussianBallDiscriminator::removeLostBalls()
{
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end();)
    {
        Matrix22 covariance = it->position.getCovariance();
        double maxCovariance = std::max(covariance.matrix[0][0], covariance.matrix[1][1]);
        double maxAllowedVariance = 16.0;

        if(maxCovariance > maxAllowedVariance)
        {
            it = gaussianBalls.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void GaussianBallDiscriminator::removeBallsOutsideField()
{
    // Add a buffer to the field length to be resilient to slightly wrong measurements
    double boundaryBuffer = 1.0;
    double boundaryHalfLength = cEnvironmentField::getInstance().getLength()/2.0 + boundaryBuffer;
    double boundaryHalfWidth = cEnvironmentField::getInstance().getWidth()/2.0 + boundaryBuffer;

    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end();)
    {
        Vector2D pos = it->position.getMean();

        if( ((std::abs(pos.x) > boundaryHalfWidth) || (std::abs(pos.y) > boundaryHalfLength)) )
        {
#ifdef DEBUG
            tprintf("OUT  pos=(%6.2f,%6.2f)", pos.x, pos.y)
#endif
            it = gaussianBalls.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void GaussianBallDiscriminator::addGaussianMeasurement(const GaussianMeasurement& measurement)
{    
    std::vector<GaussianPosVelObject>::iterator bestMatch = gaussianBalls.end();
    double bestMatchIntersection = 0.0;

    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        double intersection = it->position.getIntersection(measurement.gaussianPosition.getGaussian2D());
        bool mergeAccepted = (intersection > BALL_MERGE_THRESHOLD) && (intersection > bestMatchIntersection);

        if(mergeAccepted)
        {
            bestMatchIntersection = intersection;
            bestMatch = it;            
        }
    }

    if(bestMatch != gaussianBalls.end())
    {
        bestMatch->mergeMeasurement(measurement);
    }
    else
    {
        Gaussian2D position = measurement.gaussianPosition.getGaussian2D();
        rtime timestamp = measurement.gaussianPosition.getTimestamp();
        GaussianPosVelObject ball(position, timestamp);

        gaussianBalls.push_back(ball);
    }
}

double GaussianBallDiscriminator::getGaussianBallConfidence(const GaussianPosVelObject& ball)
{
    Vector2D eigenValues = ball.position.getCovariance().getEigenValues();

    double maxCov = std::max(eigenValues[0], eigenValues[1]);

    double confidence = 1.0/(1.0 + sqrt(maxCov));
    return confidence;
}


void GaussianBallDiscriminator::convertGaussianBallsToOutputBalls()
{
    balls.clear();

    int i=0;

    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        ballClass_t ball;

        ball.setId(i);
        ball.setTimestamp(it->timestamp);

        Vector2D pos = it->position.getMean();
        ball.setCoordinates(pos.x, pos.y, 0.0);

        Vector2D velocity = it->velocity.getMean();
        ball.setVelocities(velocity.x, velocity.y, 0.0);

        double confidence = getGaussianBallConfidence(*it);
        ball.setConfidence(confidence);
        ball.setIsValid(true);

        if(confidence >= MIN_ACCEPTED_CONFIDENCE)
        {
            balls.push_back(ball);
        }

        i++;
    }

    // sort on confidence
    std::sort(balls.begin(), balls.end());
}

void GaussianBallDiscriminator::performCalculation(rtime timeNow, const Vector2D& pos)
{
    estimateBallsMovement(timeNow);
    removeLostBalls();

    for(auto it=gaussianMeasurements->begin(); it!=gaussianMeasurements->end(); it++)
    {
        double dt = timeNow - it->gaussianPosition.getTimestamp();
        it->gaussianPosition.estimateMovement(dt, 0.01);
        addGaussianMeasurement(*it);
    }

    swapMeasurementBuffer();

    convertGaussianBallsToOutputBalls();
}

void GaussianBallDiscriminator::swapMeasurementBuffer()
{
    currentMeasurementBuffer = 1 - currentMeasurementBuffer;
    gaussianMeasurements = &(measurementsBuffer[currentMeasurementBuffer]);
    gaussianMeasurements->clear();
}

std::vector<GaussianMeasurement>* GaussianBallDiscriminator::getLastMeasurements()
{
    int lastMeasurementBuffer = 1 - currentMeasurementBuffer;
    std::vector<GaussianMeasurement>* lastMeasurements = &(measurementsBuffer[lastMeasurementBuffer]);
    return lastMeasurements;
}

std::vector<ballClass_t> GaussianBallDiscriminator::getBalls() const
{
    return balls;
}

void GaussianBallDiscriminator::getMeasurementsToSync(std::vector<ballMeasurement>& measurementsToSync)
{
    int ownRobotId = getRobotNumber();

    for(auto it = measurements.begin(); it != measurements.end(); it++)
    {
        if(it->identifier.robotID == ownRobotId)
        {
            measurementsToSync.push_back(*it);
        }
    }

    measurements.clear();
}

static diagGaussian3D createDiagnosticsGaussian(Gaussian2D gaussian2D)
{
    diagGaussian3D diagGaussian;

    Vector2D mean = gaussian2D.getMean();
    Matrix22 cov = gaussian2D.getCovariance();

    diagGaussian.mean.x = mean.x;
    diagGaussian.mean.y = mean.y;
    diagGaussian.mean.z = 0.0;
    diagGaussian.covariance.matrix[0][0] = cov.matrix[0][0];
    diagGaussian.covariance.matrix[0][1] = cov.matrix[0][1];
    diagGaussian.covariance.matrix[1][0] = cov.matrix[1][0];
    diagGaussian.covariance.matrix[1][1] = cov.matrix[1][1];

    diagGaussian.covariance.matrix[0][2] = 0.0;
    diagGaussian.covariance.matrix[1][2] = 0.0;
    diagGaussian.covariance.matrix[2][0] = 0.0;
    diagGaussian.covariance.matrix[2][1] = 0.0;
    diagGaussian.covariance.matrix[2][2] = 1.0;

    return diagGaussian;
}

static void fillMeasurementDiagData(diagWorldModel &diagnostics, std::vector<GaussianMeasurement>& measurements)
{
    diagnostics.local.gaussianBallDiscriminatorData.measurements.clear();
    for(auto it=measurements.begin(); it!=measurements.end(); it++)
    {
        diagMeasurement3D measurement;
        measurement.gaussian3d = createDiagnosticsGaussian(it->gaussianPosition.getGaussian2D());
        measurement.measurer_id = it->measurer_id;

        // diagVector2D mean = measurement.gaussian2d.mean;
        // diagMatrix22 covariance = measurement.gaussian2d.covariance;    
        // printf("M:([%f,%f],[[%f,%f],[%f,%f]]),\n", mean.x, mean.y, covariance.matrix[0][0], covariance.matrix[0][1], covariance.matrix[1][0], covariance.matrix[1][1]);
        
        diagnostics.local.gaussianBallDiscriminatorData.measurements.push_back(measurement);
    }
}

static void fillBallDiagData(diagWorldModel &diagnostics, std::vector<GaussianPosVelObject>& gaussianBalls)
{
    diagnostics.local.gaussianBallDiscriminatorData.balls.clear();
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        diagBall ball;
        ball.position = createDiagnosticsGaussian(it->position);        
        ball.velocity = createDiagnosticsGaussian(it->velocity);        
        diagnostics.local.gaussianBallDiscriminatorData.balls.push_back(ball);
    }
}

void GaussianBallDiscriminator::fillDiagnostics(diagWorldModel& diagnostics)
{
    fillMeasurementDiagData(diagnostics, *(getLastMeasurements()));
    fillBallDiagData(diagnostics, gaussianBalls);
}
