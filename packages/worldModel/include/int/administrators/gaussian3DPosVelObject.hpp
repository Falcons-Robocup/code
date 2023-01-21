// Copyright 2020-2022 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: lucas catabriga
 * Creation: 2020-02-17
 *
 * Utility class: Represent a 3D gaussian object with position and velocity
 */

#include <list>
#include <map>

#include "vector3d.hpp"
#include "matrix33.hpp"
#include "gaussian3d.hpp"

typedef std::map<uint8_t, std::list<Gaussian3DPosition> > Gaussian3DMeasurementMap;

class Gaussian3DPosVelObject
{
public:
    Gaussian3DPosVelObject(Gaussian3D position, Gaussian3D velocity, rtime timestamp);
    Gaussian3DPosVelObject(Gaussian3D position, rtime timestamp);
    
    void mergePosVelObject(Gaussian3DPosVelObject* other);
    void mergeMeasurement(const Gaussian3DMeasurement& measurement);
    void estimateMovement(double deltaTime);

    Gaussian3D getPosition() const;
    Gaussian3D getVelocity() const;
    Vector3D getAcceleration() const;
    rtime getTimestamp() const ;
    bool isFlying() const;

    void setBlacklisted(bool blacklisted);
    bool getBlacklisted() const;

    void setBestBall(bool bestBall);
    bool getBestBall() const;

    void calculateConfidence();
    double getTotalConfidence() const;
    double getPositionConfidence() const;
    double getMeasurersConfidence() const;
    double getTimeConfidence() const;
   
private:

    Gaussian3D position;
    Gaussian3D velocity;
    Vector3D acceleration;
    rtime timestamp;
    bool flying;
    bool blacklisted;
    rtime blackListTime;    
    std::list<Gaussian3DPosition> positionBuffer;
    Gaussian3DMeasurementMap measurementBuffer;
    bool bestBall;

    double total_confidence;
    double position_confidence;
    double measurers_confidence;
    double time_condifence;

    Vector3D calculateAcceleration();
    Gaussian3D calculateVelocity();
    void removeOldPositions(rtime timenow);
    void removeOldMeasurements(rtime timenow);
    double getStdDeviationTimeFactor(double deltaTime);    
    void estimateGravity(double deltaTime);

    void calculatePosConfidence();
};