// Copyright 2016-2022 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleDiscriminator.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: lucas
 */

#include "int/administrators/obstacleDiscriminator.hpp"

#include "cEnvironmentField.hpp"
#include "linalgcv.hpp"

// TODO remove debugging variable:
//static int measurement_counter = 0;

static diagGaussian2D createDiagnosticsGaussian(Gaussian2D gaussian2D)
{
    diagGaussian2D diagGaussian;

    Vector2D mean = gaussian2D.getMean();
    Matrix22 cov = gaussian2D.getCovariance();

    diagGaussian.mean.x = mean.x;
    diagGaussian.mean.y = mean.y;
    diagGaussian.covariance.matrix[0][0] = cov.matrix[0][0];
    diagGaussian.covariance.matrix[0][1] = cov.matrix[0][1];
    diagGaussian.covariance.matrix[1][0] = cov.matrix[1][0];
    diagGaussian.covariance.matrix[1][1] = cov.matrix[1][1];

    return diagGaussian;
}

obstacleDiscriminator::obstacleDiscriminator(WorldModelConfig* wmConfig) :
    wmConfig(wmConfig),
    currentMeasurementBuffer(0),
    measurements(&(measurementsBuffer[currentMeasurementBuffer])),
    obstacleMergeThreshold(wmConfig->getConfiguration().gaussianObstacles.obstacleMergeThreshold),
    minAcceptedConfidence(wmConfig->getConfiguration().gaussianObstacles.minAcceptedConfidence),
    parallelAxisDistanceFactor(wmConfig->getConfiguration().gaussianObstacles.parallelAxisDistanceFactor),
    parallelAxisOffset(wmConfig->getConfiguration().gaussianObstacles.parallelAxisOffset),
    perpendicularAxisDistanceFactor(wmConfig->getConfiguration().gaussianObstacles.perpendicularAxisDistanceFactor),
    perpendicularAxisOffset(wmConfig->getConfiguration().gaussianObstacles.perpendicularAxisOffset),
    maxAllowedVariance(wmConfig->getConfiguration().gaussianBalls.maxAllowedVariance)
{
    
}

obstacleDiscriminator::~obstacleDiscriminator()
{

}


int obstacleDiscriminator::numTrackers() const
{
    return 0; // TODO
}

double obstacleDiscriminator::getMeasurementVarianceParallelAxis(const Vector2D& measurementVec)
{
    double size = measurementVec.size();
    
    return parallelAxisOffset + size*parallelAxisDistanceFactor;
}

double obstacleDiscriminator::getMeasurementVariancePerpendicularAxis(const Vector2D& measurementVec)
{
    double size = measurementVec.size();
    
    return perpendicularAxisOffset + size*perpendicularAxisDistanceFactor;
}

GaussianMeasurement obstacleDiscriminator::gaussianMeasurementFromObstacleMeasurement(const obstacleMeasurement& measurement)
{
    Vector3D positionFcs = object2fcs(  measurement.cameraX,
                                        measurement.cameraY,
                                        measurement.cameraZ,
                                        measurement.cameraPhi,
                                        measurement.azimuth,
                                        measurement.elevation,
                                        measurement.radius);

    Vector2D obstaclePos(positionFcs.x, positionFcs.y);
    Vector2D cameraPos(measurement.cameraX, measurement.cameraY);

    Vector2D measurementVec = obstaclePos - cameraPos;
    Vector2D measurementOrthogonal(-measurementVec.y, measurementVec.x);
    double varianceParallel = getMeasurementVarianceParallelAxis(measurementVec);
    double varianceOrthogal = getMeasurementVariancePerpendicularAxis(measurementVec);

    Gaussian2D obstacleGaussianPos(obstaclePos, measurementVec, varianceParallel, measurementOrthogonal, varianceOrthogal);

    GaussianPosition positionGaussian(obstacleGaussianPos, measurement.timestamp);
    GaussianMeasurement gaussianMeasurement(positionGaussian, measurement.identifier.robotID);
    return gaussianMeasurement;
}

GaussianPosition obstacleDiscriminator::gaussianPositionFromRobotType(const robotClass_t& measurement)
{
    Vector2D obstaclePos(measurement.getX(), measurement.getY());
    Matrix22 covariance(0.1, 0.0, 0.0, 0.1);

    Gaussian2D obstacleGaussianPos(obstaclePos, covariance);

    GaussianPosition positionGaussian(obstacleGaussianPos, measurement.getTimestamp());
    return positionGaussian;
}

void obstacleDiscriminator::addMeasurement(const obstacleMeasurement& measurement)
{
    GaussianMeasurement gaussianMeasurement = gaussianMeasurementFromObstacleMeasurement(measurement);
    measurements->push_back(gaussianMeasurement);
}

std::vector<GaussianObstacle>::iterator obstacleDiscriminator::addGaussianMeasurement(const GaussianMeasurement& measurement)
{
    std::vector<GaussianObstacle>::iterator bestMatch = gaussianObstacles.end();
    double bestMatchIntersection = 0.0;
    int i=0;
    for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end(); it++)
    {
        double intersection = it->gaussianPosition.getGaussian2D().getIntersection(measurement.gaussianPosition.getGaussian2D());
        bool mergeAccepted = (intersection > obstacleMergeThreshold) && (intersection > bestMatchIntersection);

        // A robot cannot make a measurement that will merge with its own position
        bool isMergingWithItself = it->isTeammember && (measurement.measurer_id == it->robot_id);

//        printf("I:(%d,%d,%f)\n", measurement_counter, i, intersection);

        if(mergeAccepted && !isMergingWithItself)
        {
            bestMatchIntersection = intersection;
            bestMatch = it;
        }

        i++;
    }

//    Vector2D mean = measurement.gaussianPosition.getGaussian2D().getMean();
//    Matrix22 covariance = measurement.gaussianPosition.getGaussian2D().getCovariance();
//    printf("M:([%f,%f],[[%f,%f],[%f,%f]]),\n", mean.x, mean.y, covariance.matrix[0][0], covariance.matrix[0][1], covariance.matrix[1][0], covariance.matrix[1][1]);
#ifdef DEBUG
    Vector2D pos = measurement.gaussianPosition.getGaussian2D().getMean();
    tprintf("NEW  pos=(%6.2f,%6.2f)", pos.x, pos.y)
#endif

    std::vector<GaussianObstacle>::iterator insertedObstacle;
    if(bestMatch != gaussianObstacles.end())
    {
//        printf("I:Merging(%d,%ld)\n", measurement_counter, bestMatch-gaussianObstacles.begin());
        bestMatch->mergeMeasurement(measurement);

        insertedObstacle = bestMatch;
    }
    else
    {
        bool isTeammember = false;
        GaussianObstacle obstacle(measurement.gaussianPosition, isTeammember);

        gaussianObstacles.push_back(obstacle);

        insertedObstacle = gaussianObstacles.end() - 1;
    }

    return insertedObstacle;
}

void obstacleDiscriminator::removeLostObstacles()
{
    for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end();)
    {
        Matrix22 covariance = it->gaussianPosition.getGaussian2D().getCovariance();
        double maxCovariance = std::max(covariance.matrix[0][0], covariance.matrix[1][1]);

        if(maxCovariance > maxAllowedVariance)
        {
#ifdef DEBUG
            Vector2D pos = it->gaussianPosition.getGaussian2D().getMean();
            tprintf("LOST pos=(%6.2f,%6.2f)", pos.x, pos.y)
#endif
            it = gaussianObstacles.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void obstacleDiscriminator::removeObstaclesOutsideField()
{
    // Add a buffer to the field length to be resilient to slightly wrong measurements
    // and to avoid robots and the referee even when outside the field
    double boundaryBuffer = 1.0;
    double boundaryHalfLength = cEnvironmentField::getInstance().getLength()/2.0 + boundaryBuffer;
    double boundaryHalfWidth = cEnvironmentField::getInstance().getWidth()/2.0 + boundaryBuffer;

    for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end();)
    {
        Vector2D pos = it->gaussianPosition.getGaussian2D().getMean();

        if( (!it->isTeammember) && ((std::abs(pos.x) > boundaryHalfWidth) || (std::abs(pos.y) > boundaryHalfLength)) )
        {
#ifdef DEBUG
            Vector2D pos = it->gaussianPosition.getGaussian2D().getMean();
            tprintf("OUT  pos=(%6.2f,%6.2f)", pos.x, pos.y)
#endif
            it = gaussianObstacles.erase(it);
        }
        else
        {
            it++;
        }
    }
}


void obstacleDiscriminator::estimateObstaclesMovement(rtime const timeNow)
{
    for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end(); it++)
    {
        double dt = double(timeNow - it->gaussianPosition.getTimestamp());
        it->gaussianPosition.estimateMovement(dt);
    }
}

void obstacleDiscriminator::updateTeammembers(const std::vector<robotClass_t>& teamMembers)
{
    for(auto it=teamMembers.begin(); it!=teamMembers.end(); it++)
    {
        bool teammemberExists = false;
        for(auto jt=gaussianObstacles.begin(); jt!=gaussianObstacles.end(); jt++)
        {
            if(jt->isTeammember && (jt->robot_id == it->getRobotID()))
            {
                jt->gaussianPosition = gaussianPositionFromRobotType(*it);
                teammemberExists = true;
                break;
            }
        }

        if(teammemberExists == false)
        {
            // When teammember is added for the first time merge it with
            // current obstacles and transform it in a teammember

            GaussianPosition position = gaussianPositionFromRobotType(*it);
            GaussianMeasurement measurement(position, it->getRobotID());

            std::vector<GaussianObstacle>::iterator mergedObstacle = addGaussianMeasurement(measurement);
            mergedObstacle->isTeammember = true;
            mergedObstacle->robot_id = it->getRobotID();
        }
    }
}


double obstacleDiscriminator::getGaussianObstacleConfidence(const GaussianObstacle& obstacle)
{
    Vector2D eigenValues = obstacle.gaussianPosition.getGaussian2D().getCovariance().getEigenValues();

    double maxCov = std::max(eigenValues[0], eigenValues[1]);

    double confidence = 1.0/(1.0 + sqrt(maxCov));
    return confidence;
}


void obstacleDiscriminator::convertGaussianObstaclesToOutputObstacles()
{
    obstacles.clear();

    int i=0;
//    printf("Obstacles: %ld\n", gaussianObstacles.size());
    for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end(); it++)
    {
        obstacleClass_t obstacle;

        obstacle.setId(i);
        obstacle.setTimestamp(it->gaussianPosition.getTimestamp());

        Vector2D pos = it->gaussianPosition.getGaussian2D().getMean();
        obstacle.setCoordinates(pos.x, pos.y, 0.0);

        Vector2D velocity = it->gaussianPosition.getVelocity();
        obstacle.setVelocities(velocity.x, velocity.y, 0.0);

        double confidence = getGaussianObstacleConfidence(*it);
        obstacle.setConfidence(confidence);

        if(confidence >= minAcceptedConfidence && !it->isTeammember)
        {
            obstacles.push_back(obstacle);
#ifdef DEBUG
            tprintf("GOOD pos=(%6.2f,%6.2f) conf=%.1f", pos.x, pos.y, confidence)
        }
        else
        {
            tprintf("BAD  pos=(%6.2f,%6.2f) conf=%.1f", pos.x, pos.y, confidence)
#endif
        }
        
        i++;
    }
}

void obstacleDiscriminator::performCalculation(rtime const timeNow, const std::vector<robotClass_t>& teamMembers)
{
    estimateObstaclesMovement(timeNow);
    removeLostObstacles();
    updateTeammembers(teamMembers);

    for(auto it=measurements->begin(); it!=measurements->end(); it++)
    {
        double dt = timeNow - it->gaussianPosition.getTimestamp();
        it->gaussianPosition.estimateMovement(dt);
        addGaussianMeasurement(*it);
    }

    swapMeasurementBuffer();

    removeObstaclesOutsideField();

    convertGaussianObstaclesToOutputObstacles();
}

void obstacleDiscriminator::swapMeasurementBuffer()
{
    currentMeasurementBuffer = 1 - currentMeasurementBuffer;
    measurements = &(measurementsBuffer[currentMeasurementBuffer]);
    measurements->clear();
}

std::vector<GaussianMeasurement>* obstacleDiscriminator::getLastMeasurements()
{
    int lastMeasurementBuffer = 1 - currentMeasurementBuffer;
    std::vector<GaussianMeasurement>* lastMeasurements = &(measurementsBuffer[lastMeasurementBuffer]);
    return lastMeasurements;
}

std::vector<obstacleClass_t> obstacleDiscriminator::getObstacles() const
{
    return obstacles;
}

static void fillMeasurementDiagData(diagWorldModel &diagnostics, std::vector<GaussianMeasurement>& measurements)
{
    diagnostics.local.gaussianObstacleDiscriminatorData.measurements.clear();
    for(auto it=measurements.begin(); it!=measurements.end(); it++)
    {
    	diagMeasurement measurement;
    	measurement.gaussian2d = createDiagnosticsGaussian(it->gaussianPosition.getGaussian2D());
    	measurement.measurer_id = it->measurer_id;

    	diagnostics.local.gaussianObstacleDiscriminatorData.measurements.push_back(measurement);
    }
}

static void fillObstacleDiagData(diagWorldModel &diagnostics, std::vector<GaussianObstacle>& gaussianObstacles)
{
    diagnostics.local.gaussianObstacleDiscriminatorData.obstacles.clear();
    for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end(); it++)
    {
    	diagObstacle obstacle;
    	obstacle.gaussian2d = createDiagnosticsGaussian(it->gaussianPosition.getGaussian2D());
    	obstacle.isTeammember = it->isTeammember;
    	obstacle.robot_id = it->robot_id;
    	diagnostics.local.gaussianObstacleDiscriminatorData.obstacles.push_back(obstacle);
    }
}

void obstacleDiscriminator::fillDiagnostics(diagWorldModel &diagnostics)
{
    fillMeasurementDiagData(diagnostics, *(getLastMeasurements()));
    fillObstacleDiagData(diagnostics, gaussianObstacles);
}

