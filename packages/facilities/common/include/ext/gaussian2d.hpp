// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: lucas catabriga
 * Creation: 2019-09-12
 *
 * Utility class: Represent a 2D gaussian distribution
 */

#ifndef GAUSSIAN2D_HPP
#define GAUSSIAN2D_HPP

#include <deque>
#include <list>
#include <map>

#include "vector2d.hpp"
#include "matrix22.hpp"


class Gaussian2D
{
public:
    Gaussian2D()
    {
        this->mean = Vector2D(0,0);
        this->covariance = Matrix22(0,0,0,0);
    }

    Gaussian2D(Vector2D mean, Matrix22 covariance)
    {
        this->mean = mean;
        this->covariance = covariance;
    }

    // The two directions must be orthogonal
    Gaussian2D(Vector2D mean, Vector2D direction1, double varianceDir1, Vector2D direction2, double varianceDir2)
    {
        Matrix22 V(direction1, direction2);
        Matrix22 L( varianceDir1, 0,
                    0, varianceDir2);

        Matrix22 covariance = V * L * (V.getInverse());

        this->mean = mean;
        this->covariance = covariance;
    }

    Gaussian2D operator *(const Gaussian2D& other)
    {
        Matrix22 myCovInv = covariance.getInverse();
        Matrix22 otherCovInv = other.covariance.getInverse();

        Matrix22 newCovariance = (myCovInv + otherCovInv).getInverse();
        Vector2D newMean = newCovariance*(myCovInv*mean + otherCovInv*other.mean);

        return Gaussian2D(newMean, newCovariance);
    }

    Gaussian2D operator *(double factor)
    {
        Matrix22 newCovariance = covariance * (factor * factor);
        Vector2D newMean = mean * factor;

        return Gaussian2D(newMean, newCovariance);
    }

    Gaussian2D operator +(const Gaussian2D& other)
    {
        Matrix22 newCovariance = covariance + other.covariance;
        Vector2D newMean = mean + other.mean;

        return Gaussian2D(newMean, newCovariance);
    }

    Gaussian2D operator -(const Gaussian2D& other)
    {
        Matrix22 newCovariance = covariance + other.covariance;
        Vector2D newMean = mean - other.mean;

        return Gaussian2D(newMean, newCovariance);
    }

    double getIntersection(const Gaussian2D& other)
    {
        // see http://compbio.fmph.uniba.sk/vyuka/ml/old/2008/handouts/matrix-cookbook.pdf
        // Product of gaussian densities
        
        Vector2D meanDiff = (mean - other.mean);
        Matrix22 covarianceSum = covariance + other.covariance;
        double ZcExp = -0.5 * (covarianceSum.getInverse().pre_multiply(meanDiff) * meanDiff);
        double Zc = (1.0/sqrt(2.0*M_PI*covarianceSum.getDet())) * exp(ZcExp);
        return Zc;
    }

    Vector2D getMean() const
    {
        return mean;
    }

    Matrix22 getCovariance() const
    {
        return covariance;
    }

private:
    Vector2D mean;
    Matrix22 covariance;
};

class GaussianPosition
{
public:
    GaussianPosition(Gaussian2D position, rtime timestamp) :
        position(position),
        timestamp(timestamp)
    {

    }

    void estimateMovement(double deltaTime, double time_factor=1.0)
    {
        // This covariance dispersion is calculated this way
        // so that the std dev of the position increases linearly with time
        double tf = getStdDeviationTimeFactor(deltaTime, time_factor);
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
        static const int nbSamples = 30;
        this->position = position;
        this->timestamp = measurementTime;
        
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

    double getStdDeviationTimeFactor(double deltaTime, double time_factor)
    {
        double positiveDelta = std::max(deltaTime, 0.0);        
        return time_factor*positiveDelta;
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


typedef std::map<uint8_t, std::list<GaussianPosition> > GaussianMeasurementMap;
class GaussianPosVelObject
{
public:
    GaussianPosVelObject(Gaussian2D position, Gaussian2D velocity, rtime timestamp) :
                         position(position),
                         velocity(velocity),
                         timestamp(timestamp)
    {

    }

    GaussianPosVelObject(Gaussian2D position, rtime timestamp) :
                         position(position),
                         velocity(Gaussian2D(Vector2D(0,0),Matrix22(1,0,0,1))),
                         timestamp(timestamp)
    {

    }

    void removeOldMeasurements(rtime timenow)
    {
        static const double SPEED_TIME_BUFFER = 0.300;

        for(auto it = measurementBuffer.begin(); it != measurementBuffer.end(); it++)
        {
            std::list<GaussianPosition>& measurementList = it->second;
            for(auto jt = measurementList.begin(); jt != measurementList.end(); jt++)
            {
                double dt = timenow - jt->getTimestamp();
                if(dt > SPEED_TIME_BUFFER)
                {
                    jt = measurementList.erase(jt);
                }
            }
        }
    }

    Gaussian2D estimateVelocity()
    {
        int velocitiesCount = 0;
        Gaussian2D averageVelocity;

        for(GaussianMeasurementMap::iterator it = measurementBuffer.begin(); it != measurementBuffer.end(); it++)
        {
            std::list<GaussianPosition>& measurementList = it->second;
            if(measurementList.size() > 2)
            {
                GaussianPosition oldest = measurementList.front();
                GaussianPosition latest = measurementList.back();

                Gaussian2D dS = latest.getGaussian2D() - oldest.getGaussian2D();
                double dt = double(latest.getTimestamp()) - double(oldest.getTimestamp());

                Gaussian2D vel = dS * (1.0/dt);
                
                if(velocitiesCount == 0)
                {
                    averageVelocity = vel;    
                }
                else
                {
                    averageVelocity = averageVelocity * vel;    
                }
                
                velocitiesCount++;
            }
        }

        return averageVelocity;
    }

    void mergeMeasurement(const GaussianMeasurement& measurement)
    {   
        position = position * measurement.gaussianPosition.getGaussian2D();
        timestamp = measurement.gaussianPosition.getTimestamp();     
        
        measurementBuffer[measurement.measurer_id].push_back(measurement.gaussianPosition);

        removeOldMeasurements(timestamp);

        velocity = estimateVelocity();
        
    }

    void estimateMovement(double deltaTime)
    {
        // This covariance dispersion is calculated this way
        // so that the std dev of the position increases linearly with time
        double tf = getStdDeviationTimeFactor(deltaTime);
        double xStdDev = sqrt(position.getCovariance().matrix[0][0]);
        double yStdDev = sqrt(position.getCovariance().matrix[1][1]);
        double tf_sq = tf*tf;

        Vector2D dispersionMean = velocity.getMean() * deltaTime;
        Matrix22 dispersionCov(2*xStdDev*tf+tf_sq, 0.0, 0.0, 2*yStdDev*tf+tf_sq);
        Gaussian2D dispersion(dispersionMean, dispersionCov);

        position = position + dispersion;
        timestamp += deltaTime;
    }

    Gaussian2D position;
    Gaussian2D velocity;
    rtime timestamp;

private:

    GaussianMeasurementMap measurementBuffer;

    double getStdDeviationTimeFactor(double deltaTime)
    {
        double positiveDelta = std::max(deltaTime, 0.0);
        double stdDevConstant = 1.0;
        return stdDevConstant*positiveDelta;
    }
    
};


#endif