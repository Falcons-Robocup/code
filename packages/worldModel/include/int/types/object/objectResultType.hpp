// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectResultType.hpp
 *
 *  Created on: Jan 15, 2017
 *      Author: Jan Feitsma
 */

#ifndef OBJECTRESULTTYPE_HPP_
#define OBJECTRESULTTYPE_HPP_

#include <stddef.h>

class objectResultType
{
  public:
    objectResultType();
    ~objectResultType();

    void setId(const size_t id);
    void setTimestamp(const double stamp);
    void setCoordinates(const float x, const float y, const float z);
    void setVelocities(const float vx, const float vy, const float vz);

    double getTimestamp() const;
    size_t getId() const;
    float getX() const;
    float getY() const;
    float getZ() const;
    float getVX() const;
    float getVY() const;
    float getVZ() const;

  private:
    double _timestamp;
    size_t   _id;
    float _x;
    float _y;
    float _z;
    float _vx;
    float _vy;
    float _vz;
};

#endif /* OBJECTRESULTTYPE_HPP_ */

