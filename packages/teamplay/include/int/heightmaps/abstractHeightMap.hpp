// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractHeightMap.hpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

#ifndef ABSTRACTHEIGHTMAP_HPP_
#define ABSTRACTHEIGHTMAP_HPP_

#include <map>
#include <string>
#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include <opencv2/opencv.hpp>

#include "vector2d.hpp"

namespace teamplay
{

typedef std::map<std::string, std::string> parameterMap_t;

namespace heightMapValues
{
    /* The resolution is the size (in meters) of a square inside a heightmap. */
    static const float RESOLUTION = 0.25;

    static const float MIN = -100.0;
    static const float NEUTRAL = 0.0;
    static const float MAX = 100.0;
}

class heightMapField
{
public:
    void setValue (const float);
    void setValue (const float, const float, const float);
    float getValue() const;

    Point2D _center;

private:
    float _value;
};

/* A heightmap is a grid of squares */
class abstractHeightMap
{
public:
    abstractHeightMap();
    virtual ~abstractHeightMap();

    virtual void precalculate();
    virtual abstractHeightMap refine(const parameterMap_t&);
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

    cv::Mat generateOpenCVMatrix () const;
    heightMapField getOptimum() const;
    heightMapField getFieldAtCoordinate(const Point2D&) const;
    abstractHeightMap scale(const float) const;
    abstractHeightMap operator+(const abstractHeightMap&) const;

protected:
    /* reset() sets the size of the heightmap and gives every heightmapfield a center  */
    void reset();

    unsigned int getNrOfHeightMapFieldsInX() const;
    unsigned int getNrOfHeightMapFieldsInY() const;

    double angleOnPath (const Point2D& source, const Point2D& at, const Point2D& destination) const;

    /* this is the heightmap. heightmapfield _heightmap(0,0) is at field coordinate (-x, -y).
     * heightmapfield _heightmap(nrInX, nrInY) is at field coordinate (+x, +y). */
    typedef boost::numeric::ublas::matrix<heightMapField> heightMap_t;
    heightMap_t _heightMap;
};

} /* namespace teamplay */

#endif /* ABSTRACTHEIGHTMAP_HPP_ */
