// Copyright 2020-2022 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Gaussian3DBallDiscriminator.cpp
 *
 *  Created on: Nov 26, 2019
 *      Author: lucas
 */

#include "int/administrators/gaussian3DBallDiscriminator.hpp"

#include "falconsCommon.hpp"

#include "cEnvironmentField.hpp"
#include "linalgcv.hpp"


Gaussian3DBallDiscriminator::Gaussian3DBallDiscriminator(WorldModelConfig* wmConfig) :
    wmConfig(wmConfig),
    currentMeasurementBuffer(0),
    gaussianMeasurements(&(measurementsBuffer[currentMeasurementBuffer])),
    ballMergeThreshold(wmConfig->getConfiguration().gaussian3DBalls.ballMergeThreshold),
    ballVelocityMergeThreshold(wmConfig->getConfiguration().gaussian3DBalls.ballVelocityMergeThreshold),
    measurementMergeThreshold(wmConfig->getConfiguration().gaussian3DBalls.measurementMergeThreshold),
    minAcceptedConfidence(wmConfig->getConfiguration().gaussian3DBalls.minAcceptedConfidence),
    blacklistMaxVelocity(wmConfig->getConfiguration().gaussian3DBalls.blacklistMaxVelocity),
    blacklistMaxAcceleration(wmConfig->getConfiguration().gaussian3DBalls.blacklistMaxAcceleration),
    flyingMeasurementHeight(wmConfig->getConfiguration().gaussian3DBalls.flyingMeasurementHeight),
    parallelAxisDistanceFactor(wmConfig->getConfiguration().gaussian3DBalls.parallelAxisDistanceFactor),
    parallelAxisOffset(wmConfig->getConfiguration().gaussian3DBalls.parallelAxisOffset),
    parallelAxisElevationFactor(wmConfig->getConfiguration().gaussian3DBalls.parallelAxisElevationFactor),
    perpendicularAxisDistanceFactor(wmConfig->getConfiguration().gaussian3DBalls.perpendicularAxisDistanceFactor),
    perpendicularAxisOffset(wmConfig->getConfiguration().gaussian3DBalls.perpendicularAxisOffset),
    maxAllowedVariance(wmConfig->getConfiguration().gaussian3DBalls.maxAllowedVariance),
    timeFactor(wmConfig->getConfiguration().gaussian3DBalls.timeFactor)
{

}

Gaussian3DBallDiscriminator::~Gaussian3DBallDiscriminator()
{

}

double Gaussian3DBallDiscriminator::getMeasurementVarianceParallelAxis(const Vector3D& measurementVec, const ballMeasurement& measurement, double elevation)
{
    double size = vectorsize(measurementVec);
    double elevationFactor = parallelAxisElevationFactor;


    if(elevation < -M_PI/4)
    {
        elevationFactor = (M_PI/2 + elevation)/(M_PI/2);
        elevationFactor = elevationFactor*elevationFactor;
        elevationFactor = std::max(elevationFactor, 0.0);
    }

    return parallelAxisOffset + size*parallelAxisDistanceFactor*elevationFactor;
}

double Gaussian3DBallDiscriminator::getMeasurementVariancePerpendicularAxis(const Vector3D& measurementVec, const ballMeasurement& measurement)
{
    //int ownRobotId = getRobotNumber();

    double size = vectorsize(measurementVec);

    // if(measurement.identifier.robotID == ownRobotId)
    // {
    //     distanceFactor = 0.002;
    // }

    return perpendicularAxisOffset + size*perpendicularAxisDistanceFactor;
}

Vector3D Gaussian3DBallDiscriminator::getMeasurementMean(Vector3D measurementVec, const ballMeasurement& measurement)
{
    Vector3D cameraPos(measurement.cameraX,
                       measurement.cameraY,
                       measurement.cameraZ);

    Vector3D mean;
    mean = cameraPos + measurementVec;

    return mean;
}

Gaussian3DMeasurement Gaussian3DBallDiscriminator::gaussianMeasurementFromBallMeasurement(const ballMeasurement& measurement)
{
    Vector3D positionFcs = object2fcs(  measurement.cameraX,
                                        measurement.cameraY,
                                        measurement.cameraZ,
                                        measurement.cameraPhi,
                                        measurement.azimuth,
                                        measurement.elevation,
                                        measurement.radius);

    Vector3D ballPos(positionFcs.x, positionFcs.y, positionFcs.z);
    Vector3D cameraPos(measurement.cameraX, measurement.cameraY, measurement.cameraZ);

    Vector3D measurementVec = ballPos - cameraPos;
    Vector3D measurementOrthogonal1(-measurementVec.y, measurementVec.x, 0);

    // Cross product of the two previous vectors gives a vector orthogonal to both
    Vector3D measurementOrthogonal2 = measurementVec % measurementOrthogonal1;


    double varianceParallel = getMeasurementVarianceParallelAxis(measurementVec, measurement, measurement.elevation);
    double varianceOrthogal = getMeasurementVariancePerpendicularAxis(measurementVec, measurement);
    Vector3D mean = getMeasurementMean(measurementVec, measurement);

    Gaussian3D ballGaussianPos(mean, measurementVec, varianceParallel, measurementOrthogonal1, varianceOrthogal, measurementOrthogonal2, varianceOrthogal);

    Gaussian3DPosition positionGaussian(ballGaussianPos, measurement.timestamp);
    Gaussian3DMeasurement gaussianMeasurement(positionGaussian, measurement.identifier.robotID);
    return gaussianMeasurement;
}

Gaussian3DMeasurement Gaussian3DBallDiscriminator::gaussianMeasurementFromPossesionMeasurement(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp)
{
    // the perpendicularAxisOffset is arbitrarily chosen as a low value
    // for the high confidence of possession measurements
    Matrix33 covariance;
    covariance.matrix[0][0] = perpendicularAxisOffset;
    covariance.matrix[1][1] = perpendicularAxisOffset;
    covariance.matrix[2][2] = perpendicularAxisOffset;
    Gaussian3D ballGaussianPos(ball_pos, covariance);

    Gaussian3DPosition positionGaussian(ballGaussianPos, timestamp);
    Gaussian3DMeasurement gaussianMeasurement(positionGaussian, robotID);
    return gaussianMeasurement;
}

void Gaussian3DBallDiscriminator::addMeasurement(const ballMeasurement& measurement)
{    
    Gaussian3DMeasurement gaussianMeasurement = gaussianMeasurementFromBallMeasurement(measurement);

    measurements.push_back(measurement);
    gaussianMeasurements->push_back(gaussianMeasurement);
}

void Gaussian3DBallDiscriminator::addPossessionMeasurement(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp)
{
    Gaussian3DMeasurement gaussianMeasurement = gaussianMeasurementFromPossesionMeasurement(ball_pos, robotID, timestamp);

    gaussianMeasurements->push_back(gaussianMeasurement);
}

void Gaussian3DBallDiscriminator::estimateBallsMovement(rtime const timeNow)
{
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        double dt = double(timeNow - it->getTimestamp());
        it->estimateMovement(dt);
    }
}

void Gaussian3DBallDiscriminator::removeLostBalls()
{
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end();)
    {
        Matrix33 covariance = it->getPosition().getCovariance();
        double maxCovariance = std::max(std::max(covariance.matrix[0][0], covariance.matrix[1][1]),covariance.matrix[2][2]);

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

void Gaussian3DBallDiscriminator::removeBallsOutsideField()
{
    // Add a buffer to the field length to be resilient to slightly wrong measurements
    double boundaryBuffer = 1.0;
    double boundaryHalfLength = cEnvironmentField::getInstance().getLength()/2.0 + boundaryBuffer;
    double boundaryHalfWidth = cEnvironmentField::getInstance().getWidth()/2.0 + boundaryBuffer;

    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end();)
    {
        Vector3D pos = it->getPosition().getMean();

        if( (std::abs(pos.x) > boundaryHalfWidth) || (std::abs(pos.y) > boundaryHalfLength) || (pos.z < -1.0) )
        {
#ifdef DEBUG
            tprintf("OUT  pos=(%6.2f,%6.2f,%6.2f)", pos.x, pos.y, pos.z)
#endif
            it = gaussianBalls.erase(it);
        }
        else
        {
            it++;
        }
    }
}

bool Gaussian3DBallDiscriminator::isGaussianMeasurementOfFlyingBall(const Gaussian3DMeasurement& measurement)
{
    bool flying = measurement.gaussianPosition.getGaussian3D().getMean().z > flyingMeasurementHeight;
    return flying;    
}

void Gaussian3DBallDiscriminator::addGaussianMeasurement(const Gaussian3DMeasurement& measurement)
{
    std::vector<Gaussian3DPosVelObject>::iterator bestMatch = gaussianBalls.end();
    double bestMatchIntersection = 0.0;

    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        double intersection = it->getPosition().getIntersection(measurement.gaussianPosition.getGaussian3D());
        bool intersectionCondition = (intersection > measurementMergeThreshold) && (intersection > bestMatchIntersection);
        bool flying_condition = isGaussianMeasurementOfFlyingBall(measurement) == it->isFlying();

        bool mergeAccepted = intersectionCondition && flying_condition;

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
        Gaussian3D position = measurement.gaussianPosition.getGaussian3D();
        rtime timestamp = measurement.gaussianPosition.getTimestamp();
        Gaussian3DPosVelObject ball(position, timestamp);

        gaussianBalls.push_back(ball);
    }
}

void Gaussian3DBallDiscriminator::blacklistFakeBalls()
{
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        if(it->isFlying())
        {
            // Vector3D velocity = it->getVelocity().getMean();

            // bool blacklisted = sqrt(velocity.x*velocity.x + velocity.y*velocity.y) < BLACKLIST_MAX_VELOCITY;
            // it->setBlacklisted(blacklisted);

            Vector3D acceleration = it->getAcceleration();

            bool blacklisted = acceleration.z > blacklistMaxAcceleration;
            it->setBlacklisted(blacklisted);
        }
        else
        {
            it->setBlacklisted(false);
        }
    }
}

void Gaussian3DBallDiscriminator::mergeBalls()
{
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        Gaussian3D position_it = it->getPosition();

        for(auto jt=(it+1); jt!=gaussianBalls.end();)
        {
            Gaussian3D position_jt = jt->getPosition();
            double intersection_pos = position_it.getIntersection(position_jt);

            Gaussian3D velocity_it = it->getVelocity();
            Gaussian3D velocity_jt = jt->getVelocity();
            double intersection_vel = velocity_it.getIntersection(velocity_jt);

            bool posIntersectionCondition = intersection_pos > ballMergeThreshold;
            bool velIntersectionCondition = intersection_vel > ballVelocityMergeThreshold;
            bool flying_condition = it->isFlying() == jt->isFlying();

            if(posIntersectionCondition && velIntersectionCondition && flying_condition)
            {
                it->mergePosVelObject(&(*jt));

                jt = gaussianBalls.erase(jt);
                continue;
            }

            jt++;
        }
    }
}

void Gaussian3DBallDiscriminator::convertGaussianBallsToOutputBalls()
{
    balls.clear();

    int i=0;

    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        ballClass_t ball;

        ball.setId(i);
        ball.setTimestamp(it->getTimestamp());

        Vector3D pos = it->getPosition().getMean();
        ball.setCoordinates(pos.x, pos.y, 0.0);

        Vector3D velocity = it->getVelocity().getMean();
        ball.setVelocities(velocity.x, velocity.y, 0.0);

        it->calculateConfidence();
        double confidence = it->getTotalConfidence();
        ball.setConfidence(confidence);
        ball.setIsValid(true);

        if(confidence >= minAcceptedConfidence && (!it->getBlacklisted()))
        {
            balls.push_back(ball);
        }

        i++;
        it->setBestBall(false);
    }

    // sort on confidence
    std::sort(balls.begin(), balls.end());

    if(balls.size() > 0)
    {
        int bestBallIndex = balls[0].getId();
        gaussianBalls[bestBallIndex].setBestBall(true);
    }
}

void Gaussian3DBallDiscriminator::performCalculation(rtime timeNow, const Vector2D& pos)
{
    estimateBallsMovement(timeNow);
    removeLostBalls();

    for(auto it=gaussianMeasurements->begin(); it!=gaussianMeasurements->end(); it++)
    {
        double dt = timeNow - it->gaussianPosition.getTimestamp();
        it->gaussianPosition.estimateMovement(dt, timeFactor);
        addGaussianMeasurement(*it);
    }

    swapMeasurementBuffer();

    removeBallsOutsideField();
    blacklistFakeBalls();
    mergeBalls();
    convertGaussianBallsToOutputBalls();
}

void Gaussian3DBallDiscriminator::swapMeasurementBuffer()
{
    currentMeasurementBuffer = 1 - currentMeasurementBuffer;
    gaussianMeasurements = &(measurementsBuffer[currentMeasurementBuffer]);
    gaussianMeasurements->clear();
}

std::vector<Gaussian3DMeasurement>* Gaussian3DBallDiscriminator::getLastMeasurements()
{
    int lastMeasurementBuffer = 1 - currentMeasurementBuffer;
    std::vector<Gaussian3DMeasurement>* lastMeasurements = &(measurementsBuffer[lastMeasurementBuffer]);
    return lastMeasurements;
}

std::vector<ballClass_t> Gaussian3DBallDiscriminator::getBalls() const
{
    return balls;
}

void Gaussian3DBallDiscriminator::getMeasurementsToSync(std::vector<ballMeasurement>& measurementsToSync)
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

static diagGaussian3D createDiagnosticsGaussian(Gaussian3D gaussian3D)
{
    diagGaussian3D diagGaussian;

    Vector3D mean = gaussian3D.getMean();
    Matrix33 cov = gaussian3D.getCovariance();

    diagGaussian.mean.x = mean.x;
    diagGaussian.mean.y = mean.y;
    diagGaussian.mean.z = mean.z;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            diagGaussian.covariance.matrix[i][j] = cov.matrix[i][j];        
        }
    }
    
    return diagGaussian;
}

static void fillMeasurementDiagData(diagWorldModel &diagnostics, std::vector<Gaussian3DMeasurement>& measurements)
{
    diagnostics.local.gaussianBallDiscriminatorData.measurements.clear();
    for(auto it=measurements.begin(); it!=measurements.end(); it++)
    {
        diagMeasurement3D measurement;
        measurement.gaussian3d = createDiagnosticsGaussian(it->gaussianPosition.getGaussian3D());
        measurement.measurer_id = it->measurer_id;
        
        diagnostics.local.gaussianBallDiscriminatorData.measurements.push_back(measurement);
    }
}

static void fillBallDiagData(diagWorldModel &diagnostics, std::vector<Gaussian3DPosVelObject>& gaussianBalls)
{
    diagnostics.local.gaussianBallDiscriminatorData.balls.clear();
    for(auto it=gaussianBalls.begin(); it!=gaussianBalls.end(); it++)
    {
        diagBall ball;
        ball.position = createDiagnosticsGaussian(it->getPosition());        
        ball.velocity = createDiagnosticsGaussian(it->getVelocity());
        ball.blacklisted = it->getBlacklisted();
        ball.bestBall = it->getBestBall();
        
        ball.confidence.total_confidence = it->getTotalConfidence();
        ball.confidence.position_confidence = it->getPositionConfidence();
        ball.confidence.measurers_confidence = it->getMeasurersConfidence();
        ball.confidence.time_condifence = it->getTimeConfidence();        

        diagnostics.local.gaussianBallDiscriminatorData.balls.push_back(ball);
    }
}

void Gaussian3DBallDiscriminator::fillDiagnostics(diagWorldModel& diagnostics)
{
    fillMeasurementDiagData(diagnostics, *(getLastMeasurements()));
    fillBallDiagData(diagnostics, gaussianBalls);
}
