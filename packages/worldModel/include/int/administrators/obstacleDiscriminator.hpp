 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

class GaussianPosition
{
public:
    GaussianPosition(Gaussian2D position, rtime timestamp) :
        position(position),
        timestamp(timestamp)
    {

    }

    void estimateMovement(double deltaTime)
    {
        // This covariance dispersion is calculated this way
        // so that the std dev of the position increases linearly with time
        double tf = getStdDeviationTimeFactor(deltaTime);
        double xStdDev = sqrt(position.getCovariance().matrix[0][0]);
        double yStdDev = sqrt(position.getCovariance().matrix[1][1]);
        double tf_sq = tf*tf;

        Vector2D dispersionMean(0.0, 0.0);
        Matrix22 dispersionCov(2*xStdDev*tf+tf_sq, 0.0, 0.0, 2*yStdDev*tf+tf_sq);
        Gaussian2D dispersion(dispersionMean, dispersionCov);

        position = position + dispersion;
        timestamp += deltaTime;
    }

    void setGaussian2D(Gaussian2D position, rtime measurementTime)
    {
        
        const int nbSamples = 30;
        this->position = position;
        
        positions.push_back(position.getMean());
        timestamps.push_back(measurementTime);
        
        if (positions.size() > nbSamples)
        {
            positions.pop_front();
            timestamps.pop_front();
        }
        
    }
    
    Vector2D getVelocity()
    {
        int  nbSamples = 0;
        Vector2D velocity;
        
        for (size_t i = 1; i < positions.size(); ++i)
        {
            double dt = timestamps[i] - timestamps[i-1];
            if ( dt > 1e-6)
            {
                velocity += (positions[i] - positions[i-1]) / dt;
                nbSamples++;
            }
        }
        
        if (nbSamples > 0)
        {
            velocity /= nbSamples;
        }
        
        return velocity;
    }

    Gaussian2D getGaussian2D() const
    {
        return position;
    }

    rtime getTimestamp() const
    {
        return timestamp;
    }


private:
    Gaussian2D position;
    rtime timestamp;
    
    std::deque<Vector2D> positions;
    std::deque<rtime> timestamps;

    double getStdDeviationTimeFactor(double deltaTime)
    {
        double positiveDelta = std::max(deltaTime, 0.0);
        double stdDevConstant = 1.0;
        return stdDevConstant*positiveDelta;
    }
};

class GaussianMeasurement
{
public:
    GaussianMeasurement(GaussianPosition gaussianPosition, uint8_t measurer_id) :
        gaussianPosition(gaussianPosition),
        measurer_id(measurer_id)
    {

    }

    GaussianPosition gaussianPosition;
    uint8_t measurer_id;

};

class GaussianObstacle
{
public:
    GaussianObstacle(GaussianPosition gaussianPosition, bool isTeammember) :
        gaussianPosition(gaussianPosition),
        isTeammember(isTeammember),
        robot_id(-1)
    {

    }

    void mergeMeasurement(const GaussianMeasurement& measurement)
    {
        Gaussian2D mergedPosition = gaussianPosition.getGaussian2D() * measurement.gaussianPosition.getGaussian2D();
        gaussianPosition.setGaussian2D(mergedPosition, measurement.gaussianPosition.getTimestamp());
    }

    GaussianPosition gaussianPosition;
    bool isTeammember;
    uint8_t robot_id; // only valid if it is a team member
};

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
