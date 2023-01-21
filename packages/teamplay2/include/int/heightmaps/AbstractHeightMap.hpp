// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractHeightMap.hpp
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

namespace HeightMapValues
{
    /* The resolution is the size (in meters) of a square inside a heightmap. */
    static const float RESOLUTION = 0.25;

    static const float MIN = -100.0;
    static const float NEUTRAL = 0.0;
    static const float MAX = 100.0;
}

class HeightMapField
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
class AbstractHeightMap
{
public:
    AbstractHeightMap();
    virtual ~AbstractHeightMap();

    virtual void precalculate();
    virtual AbstractHeightMap refine();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

    void resetHeightmapPrecalculation();

    cv::Mat generateOpenCVMatrix () const;
    HeightMapField getOptimum() const;
    HeightMapField getFieldAtCoordinate(const Point2D&) const;
    AbstractHeightMap scale(const float) const;
    AbstractHeightMap operator+(const AbstractHeightMap&) const;

protected:
    /* reset() sets the size of the heightmap and gives every heightmapfield a center  */
    void reset();

    unsigned int getNrOfHeightMapFieldsInX() const;
    unsigned int getNrOfHeightMapFieldsInY() const;

    double angleOnPath (const Point2D& source, const Point2D& at, const Point2D& destination) const;

    void getOptimum(unsigned int& i, unsigned int& j) const;

    /* this is the heightmap. heightmapfield _heightmap(0,0) is at field coordinate (-x, -y).
     * heightmapfield _heightmap(nrInX, nrInY) is at field coordinate (+x, +y). */
    typedef boost::numeric::ublas::matrix<HeightMapField> HeightMap_t;
    HeightMap_t _heightMap;

    bool _heightmapPrecalculated;
};

} /* namespace teamplay */

#endif /* ABSTRACTHEIGHTMAP_HPP_ */
