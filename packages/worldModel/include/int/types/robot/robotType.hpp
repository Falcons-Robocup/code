// Copyright 2016-2019 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotType.hpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTTYPE_HPP_
#define ROBOTTYPE_HPP_

#include <stdint.h>

#include "int/types/coordinateType.hpp"

class robotClass_t
{
    public:
        robotClass_t();
        ~robotClass_t();

        static bool sortOnIncreasingRobotID(const robotClass_t& objA, const robotClass_t& objB)
        {
            return objA.getRobotID() < objB.getRobotID();
        }

        void setRobotID(const uint8_t robotID);
        void setTimestamp(const double stamp);
        void setCoordinateType(const coordinateType coordinate);
        void setCoordinates(const float x, const float y, const float theta);
        void setVelocities(const float vx, const float vy, const float vtheta);
        void setBallPossession(bool bp);

        uint8_t getRobotID() const;
        double getTimestamp() const;
        coordinateType getCoordindateType() const;
        float getX() const;
        float getY() const;
        float getTheta() const;
        float getVX() const;
        float getVY() const;
        float getVTheta() const;
        bool getBallPossession() const;

    private:
        uint8_t _robotID;
        double _timestamp;
        coordinateType _coordinates;
        float _x;
        float _y;
        float _theta;
        float _vx;
        float _vy;
        float _vtheta;
        bool _ballPossession;
};

#endif /* ROBOTTYPE_HPP_ */
