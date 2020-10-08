 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleDiscriminator.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: lucas
 */

#include "int/administrators/obstacleDiscriminator.hpp"

#include "linalgcv.hpp"

// TODO remove debugging variable:
//static int measurement_counter = 0;

const static double MIN_ACCEPTED_CONFIDENCE = 0.4; // TODO make configurable


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

obstacleDiscriminator::obstacleDiscriminator() :
    currentMeasurementBuffer(0),
    measurements(&(measurementsBuffer[currentMeasurementBuffer])),
    obstacleMergeThreshold(0.01)
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
    double distanceFactor = 0.08;
    double size = measurementVec.size();
    double offset = 0.0;

    return offset + size*distanceFactor;
}

double obstacleDiscriminator::getMeasurementVariancePerpendicularAxis(const Vector2D& measurementVec)
{
    double distanceFactor = 0.12;
    double size = measurementVec.size();
    double offset = 0.01;

    return offset + size*distanceFactor;
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
        double maxAllowedVariance = 16.0;

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
    double boundaryHalfLength = 9.0; // TODO use cEnvironmentField, do this filtering at administrator level, not down here, and allow configurable margins for instance when taking a throwin. Note that referee often walks outside the lines, we should at least attempt to avoid him/her ...
    double boundaryHalfWidth = 6.0;

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

        if(confidence >= MIN_ACCEPTED_CONFIDENCE && !it->isTeammember)
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

//        Matrix22 cov = it->gaussianPosition.getGaussian2D().getCovariance();
//        printf("M:([%f,%f],[[%f,%f],[%f,%f]],[%f,%f]),\n", pos.x, pos.y, cov.matrix[0][0], cov.matrix[0][1], cov.matrix[1][0], cov.matrix[1][1], velocity.x, velocity.y);

        i++;
    }
}

void obstacleDiscriminator::performCalculation(rtime const timeNow, const std::vector<robotClass_t>& teamMembers)
{
    estimateObstaclesMovement(timeNow);
    removeLostObstacles();
    updateTeammembers(teamMembers);

//    printf("Measurements: %ld\n", measurements->size() + 1);
//
//    // This section is to draw the self position as a measurement
//    if(measurements->size() > 0)
//    {
//        for(auto it=gaussianObstacles.begin(); it!=gaussianObstacles.end(); it++)
//        {
//            if(it->isTeammember && it->robot_id == (*measurements)[0].measurer_id)
//            {
//                Vector2D pos = it->gaussianPosition.getGaussian2D().getMean();
//                Matrix22 cov = it->gaussianPosition.getGaussian2D().getCovariance();
//                printf("M:([%f,%f],[[%f,%f],[%f,%f]]),\n", pos.x, pos.y, cov.matrix[0][0], cov.matrix[0][1], cov.matrix[1][0], cov.matrix[1][1]);
//                break;
//            }
//        }
//    }

//    measurement_counter = 1;
    for(auto it=measurements->begin(); it!=measurements->end(); it++)
    {
        double dt = timeNow - it->gaussianPosition.getTimestamp();
        it->gaussianPosition.estimateMovement(dt);
        addGaussianMeasurement(*it);
//        measurement_counter++;
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

