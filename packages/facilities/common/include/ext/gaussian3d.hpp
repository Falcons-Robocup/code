// Copyright 2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: lucas catabriga
 * Creation: 2020-02-09
 *
 * Utility class: Represent a 3D gaussian distribution
 */

#ifndef GAUSSIAN3D_HPP
#define GAUSSIAN3D_HPP

#include "vector3d.hpp"
#include "matrix33.hpp"


class Gaussian3D
{
public:
    Gaussian3D()
    {
        this->mean = Vector3D(0,0,0);
        this->covariance = Matrix33();
    }

    Gaussian3D(Vector3D mean, Matrix33 covariance)
    {
        this->mean = mean;
        this->covariance = covariance;
    }

    // All directions must be orthogonal
    Gaussian3D(Vector3D mean, Vector3D direction1, double varianceDir1, Vector3D direction2, double varianceDir2, Vector3D direction3, double varianceDir3)
    {
        Matrix33 V(direction1, direction2, direction3);
        Matrix33 L( Vector3D(varianceDir1, 0, 0),
                    Vector3D(0, varianceDir2, 0),
                    Vector3D(0, 0, varianceDir3));

        Matrix33 covariance = V * L * (V.getInverse());

        this->mean = mean;
        this->covariance = covariance;
    }

    Gaussian3D operator *(const Gaussian3D& other)
    {
        Matrix33 myCovInv = covariance.getInverse();
        Matrix33 otherCovInv = other.covariance.getInverse();

        Matrix33 newCovariance = (myCovInv + otherCovInv).getInverse();
        Vector3D newMean = newCovariance*(myCovInv*mean + otherCovInv*other.mean);

        return Gaussian3D(newMean, newCovariance);
    }

    Gaussian3D operator *(double factor)
    {
        Matrix33 newCovariance = covariance * (factor * factor);
        Vector3D newMean = mean * factor;

        return Gaussian3D(newMean, newCovariance);
    }

    Gaussian3D operator +(const Gaussian3D& other)
    {
        Matrix33 newCovariance = covariance + other.covariance;
        Vector3D newMean = mean + other.mean;

        return Gaussian3D(newMean, newCovariance);
    }

    Gaussian3D operator -(const Gaussian3D& other)
    {
        Matrix33 newCovariance = covariance + other.covariance;
        Vector3D newMean = mean - other.mean;

        return Gaussian3D(newMean, newCovariance);
    }

    double getIntersection(const Gaussian3D& other)
    {
        // see http://compbio.fmph.uniba.sk/vyuka/ml/old/2008/handouts/matrix-cookbook.pdf
        // Product of gaussian densities

        Vector3D meanDiff = (mean - other.mean);
        Matrix33 covarianceSum = covariance + other.covariance;
        double ZcExp = -0.5 * (covarianceSum.getInverse().pre_multiply(meanDiff) * meanDiff);
        double Zc = (1.0/sqrt(2.0*M_PI*covarianceSum.getDet())) * exp(ZcExp);
        return Zc;
    }

    Vector3D getMean() const
    {
        return mean;
    }

    Matrix33 getCovariance() const
    {
        return covariance;
    }

private:
    Vector3D mean;
    Matrix33 covariance;
};

class Gaussian3DPosition
{
public:
    Gaussian3DPosition(Gaussian3D position, rtime timestamp) :
        position(position),
        timestamp(timestamp)
    {

    }

    void estimateMovement(double deltaTime, double time_factor=1.0)
    {
        // This covariance dispersion is calculated this way
        // so that the std dev of the position increases linearly with time
        double tf = getStdDeviationTimeFactor(deltaTime, time_factor);

        Matrix33 covariance = position.getCovariance();
        double xStdDev = sqrt(covariance.matrix[0][0]);
        double yStdDev = sqrt(covariance.matrix[1][1]);
        double zStdDev = sqrt(covariance.matrix[2][2]);
        double tf_sq = tf*tf;

        Vector3D dispersionMean(0.0, 0.0, 0.0);
        Matrix33 dispersionCov( Vector3D(2*xStdDev*tf+tf_sq, 0, 0),
                                Vector3D(0, 2*yStdDev*tf+tf_sq, 0),
                                Vector3D(0, 0, 2*zStdDev*tf+tf_sq));
        Gaussian3D dispersion(dispersionMean, dispersionCov);

        position = position + dispersion;
        timestamp += deltaTime;
    }

    void setGaussian3D(Gaussian3D position, rtime measurementTime)
    {        
        this->position = position;
        this->timestamp = measurementTime;
    }
    
    Gaussian3D getGaussian3D() const
    {
        return position;
    }

    rtime getTimestamp() const
    {
        return timestamp;
    }


private:
    Gaussian3D position;
    rtime timestamp;
    
    double getStdDeviationTimeFactor(double deltaTime, double time_factor)
    {
        double positiveDelta = std::max(deltaTime, 0.0);        
        return time_factor*positiveDelta;
    }
};


class Gaussian3DMeasurement
{
public:
    Gaussian3DMeasurement(Gaussian3DPosition gaussianPosition, uint8_t measurer_id) :
        gaussianPosition(gaussianPosition),
        measurer_id(measurer_id)
    {

    }

    Gaussian3DPosition gaussianPosition;
    uint8_t measurer_id;
};

#endif