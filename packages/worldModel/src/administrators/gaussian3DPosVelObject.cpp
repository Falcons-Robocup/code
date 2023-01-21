// Copyright 2020-2022 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: lucas catabriga
 * Creation: 2020-02-17
 *
 * Utility class: Represent a 3D gaussian object with position and velocity
 */

#include "LeastSquaresInterpolator.hpp"

#include "int/administrators/gaussian3DPosVelObject.hpp"

static const Vector3D GRAVITY(0.0, 0.0, -9.807);
static const double SPEED_TIME_BUFFER = 0.300;
static const double POSITION_TIME_BUFFER = 0.300;
static const double MIN_FLYING_HEIGHT = 0.4;
static const double DISPERTION_TIME_FACTOR = 1.0;
static const double BLACKLIST_FADE_DELAY = 0.0;

static const double MEASURER_CONFIDENCE_FACTOR = 0.3;


Gaussian3DPosVelObject::Gaussian3DPosVelObject(Gaussian3D position, Gaussian3D velocity, rtime timestamp) :
                       position(position),
                       velocity(velocity),
                       timestamp(timestamp),
                       flying(false),
                       blacklisted(false),
                       bestBall(false)
{

}

Gaussian3DPosVelObject::Gaussian3DPosVelObject(Gaussian3D position, rtime timestamp) :
                       position(position),
                       velocity(Gaussian3D(Vector3D(0,0,0),Matrix33::I())),
                       timestamp(timestamp),
                       flying(false),
                       blacklisted(false),
                       bestBall(false)
{

}

void Gaussian3DPosVelObject::removeOldPositions(rtime timenow)
{
    for(auto it = positionBuffer.begin(); it != positionBuffer.end();)
    {
        double dt = timenow - it->getTimestamp();
        if(dt > POSITION_TIME_BUFFER)
        {
            it = positionBuffer.erase(it);
        }
        else
        {
            it++;
        }
    }    
}

void Gaussian3DPosVelObject::removeOldMeasurements(rtime timenow)
{
    for(auto it = measurementBuffer.begin(); it != measurementBuffer.end(); it++)
    {
        std::list<Gaussian3DPosition>& measurementList = it->second;
        for(auto jt = measurementList.begin(); jt != measurementList.end();)
        {
            double dt = timenow - jt->getTimestamp();
            if(dt > SPEED_TIME_BUFFER)
            {
                jt = measurementList.erase(jt);
            }
            else
            {
                jt++;
            }
        }
    }
}

Gaussian3D Gaussian3DPosVelObject::calculateVelocity()
{
    // Velocity is calculated from the point of view of each of the measurer robots
    // and then averaged for all the robots
    // This is done because if there is a small bias in each robot measurement
    // this could lead to a big difference in the final calculated velocity, if positions
    // from two different robots are used
    // As the velocity is also calculated as a gaussian, the velocity measured from positions
    // with a greater confidence will have a greater impact on the final average velocity

    int velocitiesCount = 0;
    Gaussian3D averageVelocity;

    for(Gaussian3DMeasurementMap::iterator it = measurementBuffer.begin(); it != measurementBuffer.end(); it++)
    {
        std::list<Gaussian3DPosition>& measurementList = it->second;
        if(measurementList.size() > 2)
        {
            Gaussian3DPosition oldest = measurementList.front();
            Gaussian3DPosition latest = measurementList.back();

            Gaussian3D dS = latest.getGaussian3D() - oldest.getGaussian3D();
            double dt = double(latest.getTimestamp()) - double(oldest.getTimestamp());

            if(dt > 1e-3)
            {
                Gaussian3D vel = dS * (1.0/dt);

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
    }

    return averageVelocity;
}

Vector3D Gaussian3DPosVelObject::calculateAcceleration()
{
    Vector3D acceleration;

    int order = 2;
    LeastSquaresInterpolator interpolator_x(order);
    LeastSquaresInterpolator interpolator_y(order);
    LeastSquaresInterpolator interpolator_z(order);

    if(positionBuffer.size() >= 3)
    {
        double t0 = positionBuffer.begin()->getTimestamp().toDouble();

        for(auto it = positionBuffer.begin(); it != positionBuffer.end(); it++)
        {
            double t = t0 - it->getTimestamp().toDouble();
            double x = it->getGaussian3D().getMean().x;
            double y = it->getGaussian3D().getMean().y;
            double z = it->getGaussian3D().getMean().z;

            interpolator_x.feed(t, x);
            interpolator_y.feed(t, y);
            interpolator_z.feed(t, z);
        }

        std::vector<double> x_poly = interpolator_x.calculate_polynomial();
        std::vector<double> y_poly = interpolator_y.calculate_polynomial();
        std::vector<double> z_poly = interpolator_z.calculate_polynomial();

        acceleration.x = x_poly[2];
        acceleration.y = y_poly[2];
        acceleration.z = z_poly[2];
    }

    return acceleration;
}

void Gaussian3DPosVelObject::mergeMeasurement(const Gaussian3DMeasurement& measurement)
{   
    position = position * measurement.gaussianPosition.getGaussian3D();
    timestamp = measurement.gaussianPosition.getTimestamp();     
    
    positionBuffer.push_back(Gaussian3DPosition(position, timestamp));
    measurementBuffer[measurement.measurer_id].push_back(measurement.gaussianPosition);

    removeOldPositions(timestamp);
    removeOldMeasurements(timestamp);

    velocity = calculateVelocity();
    acceleration = calculateAcceleration();
}

// compares positions based on timestamp
static bool timestampComparison(const Gaussian3DPosition& first, const Gaussian3DPosition& second)
{
    return double(first.getTimestamp()) < double(second.getTimestamp());
}

void Gaussian3DPosVelObject::mergePosVelObject(Gaussian3DPosVelObject* other)
{
    position = position * other->position;

    positionBuffer.merge(other->positionBuffer, timestampComparison);

    for(auto it=other->measurementBuffer.begin(); it!=other->measurementBuffer.end(); it++)
    {
        std::list<Gaussian3DPosition>& measurementList = it->second;

        measurementBuffer[it->first].merge(measurementList, timestampComparison);
    }

    removeOldPositions(timestamp);
    removeOldMeasurements(timestamp);

    velocity = calculateVelocity();
    acceleration = calculateAcceleration();
}

void Gaussian3DPosVelObject::estimateMovement(double deltaTime)
{
    // not estimating gravity because this made static fake balls
    // more difficult to handle
    //estimateGravity(deltaTime);

    // This covariance dispersion is calculated this way
    // so that the std dev of the position increases linearly with time
    double tf = getStdDeviationTimeFactor(deltaTime);

    Matrix33 covariance = position.getCovariance();
    double xStdDev = sqrt(covariance.matrix[0][0]);
    double yStdDev = sqrt(covariance.matrix[1][1]);
    double zStdDev = sqrt(covariance.matrix[2][2]);
    double tf_sq = tf*tf;

    Vector3D dispersionMean = velocity.getMean() * deltaTime;
    Matrix33 dispersionCov( Vector3D(2*xStdDev*tf+tf_sq, 0, 0),
                            Vector3D(0, 2*yStdDev*tf+tf_sq, 0),
                            Vector3D(0, 0, 2*zStdDev*tf+tf_sq));
    Gaussian3D dispersion(dispersionMean, dispersionCov);

    position = position + dispersion;
    timestamp += deltaTime;

    flying = (position.getMean().z > MIN_FLYING_HEIGHT);
}

void Gaussian3DPosVelObject::estimateGravity(double deltaTime)
{
    if(flying)
    {
        Vector3D gravityMean = GRAVITY*deltaTime;
        Matrix33 gravityCovariance; // zero matrix
        Gaussian3D gravityDelta(gravityMean, gravityCovariance);

        velocity = velocity + gravityDelta;
    }
}

double Gaussian3DPosVelObject::getStdDeviationTimeFactor(double deltaTime)
{
    double positiveDelta = std::max(deltaTime, 0.0);
    return DISPERTION_TIME_FACTOR*positiveDelta;
}

void Gaussian3DPosVelObject::setBlacklisted(bool blacklisted)
{
    this->blacklisted = blacklisted;

    if(blacklisted)
    {
        blackListTime = timestamp;
    }
}

bool Gaussian3DPosVelObject::getBlacklisted() const
{
    if(!blacklisted)
    {
        if((timestamp.toDouble() - blackListTime.toDouble()) < BLACKLIST_FADE_DELAY)
        {
            return true;
        }
    }

    return blacklisted;
}

bool Gaussian3DPosVelObject::isFlying() const
{
    return flying;
}

Gaussian3D Gaussian3DPosVelObject::getPosition() const
{
    return position;
}

Gaussian3D Gaussian3DPosVelObject::getVelocity() const
{
    return velocity;
}

Vector3D Gaussian3DPosVelObject::getAcceleration() const
{
    return acceleration;
}

rtime Gaussian3DPosVelObject::getTimestamp() const
{
    return timestamp;
}

void Gaussian3DPosVelObject::setBestBall(bool bestBall)
{
    this->bestBall = bestBall;
}

bool Gaussian3DPosVelObject::getBestBall() const
{
    return bestBall;
}

static double calculatePositionConfidence(const Gaussian3D& position)
{
    Vector3D eigenValues = position.getCovariance().getEigenValues();

    double maxCov = std::max(eigenValues[0], std::max(eigenValues[1], eigenValues[2]));

    double confidence = 1.0/(1.0 + sqrt(maxCov));
    return confidence;
}

static double calculateMeasurersConfidence(const Gaussian3DMeasurementMap& measurementBuffer)
{
    int num_recent_measurers = 0;

    for(auto it=measurementBuffer.begin(); it!=measurementBuffer.end(); it++)
    {
        const std::list<Gaussian3DPosition>& measurementList = it->second;

        if(measurementList.size() > 0)
        {
            num_recent_measurers++;
        }
    }

    double confidence = 1.0 - (1.0/(num_recent_measurers + MEASURER_CONFIDENCE_FACTOR));

    return confidence;
}

static double calculateFlyingConfidence(bool flying)
{
    double confidence = 1.0;

    if(flying)
    {
        confidence = 0.1;
    }
    
    return confidence;
}

void Gaussian3DPosVelObject::calculateConfidence()
{
    position_confidence = calculatePositionConfidence(position);    
    measurers_confidence = calculateMeasurersConfidence(measurementBuffer);
    time_condifence = 1.0;
    double flying_confidence = calculateFlyingConfidence(flying);

    total_confidence = position_confidence * measurers_confidence * time_condifence * flying_confidence;
}

double Gaussian3DPosVelObject::getTotalConfidence() const
{
    return total_confidence;
}

double Gaussian3DPosVelObject::getPositionConfidence() const
{
    return position_confidence;
}

double Gaussian3DPosVelObject::getMeasurersConfidence() const
{
    return measurers_confidence;
}

double Gaussian3DPosVelObject::getTimeConfidence() const
{
    return time_condifence;
}

