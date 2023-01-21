// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractHeightMap.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/AbstractHeightMap.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "boost/format.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static int fieldsize = 10;

void HeightMapField::setValue (const float value)
{
    setValue(value, HeightMapValues::MIN, HeightMapValues::MAX);
}

void HeightMapField::setValue (const float value, const float floor, const float ceil)
{
    if ((floor <= value) && (value <= ceil))
    {
        _value = value;
    }

    if (value < floor)
    {
        _value = floor;
    }

    if (value > ceil)
    {
        _value = ceil;
    }
}

float HeightMapField::getValue() const
{
    return _value;
}

AbstractHeightMap::AbstractHeightMap()
{
    reset();
}

AbstractHeightMap::~AbstractHeightMap() { }

unsigned int AbstractHeightMap::getNrOfHeightMapFieldsInX() const
{
    return _heightMap.size1();
}

unsigned int AbstractHeightMap::getNrOfHeightMapFieldsInY() const
{
    return _heightMap.size2();
}

void AbstractHeightMap::reset()
{
    _heightMap.resize((int)ceil(FieldDimensionsStore::getFieldDimensions().getWidth() / HeightMapValues::RESOLUTION),
                    (int)ceil(FieldDimensionsStore::getFieldDimensions().getLength() / HeightMapValues::RESOLUTION));

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto center = Point2D( (i - floor(getNrOfHeightMapFieldsInX() / 2)) * HeightMapValues::RESOLUTION,
                                   (j - floor(getNrOfHeightMapFieldsInY() / 2)) * HeightMapValues::RESOLUTION);
            _heightMap(i, j)._center = center;
            _heightMap(i, j).setValue(HeightMapValues::NEUTRAL);
        }
    }

    _heightmapPrecalculated = false;
}

void AbstractHeightMap::precalculate()
{
    throw std::runtime_error("Not implemented");
}

AbstractHeightMap AbstractHeightMap::refine()
{
    return *this;
}

std::string AbstractHeightMap::getDescription() const
{
    throw std::runtime_error("Not implemented");
}

std::string AbstractHeightMap::getFilename() const
{
    throw std::runtime_error("Not implemented");
}

void AbstractHeightMap::resetHeightmapPrecalculation()
{
    _heightmapPrecalculated = false;
}

HeightMapField AbstractHeightMap::getOptimum() const
{
    HeightMapField optimum;
    optimum.setValue(std::numeric_limits<float>::min());

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            if (_heightMap(i, j).getValue() > optimum.getValue())
            {
                optimum = _heightMap(i, j);
            }
        }
    }

    return optimum;
}

HeightMapField AbstractHeightMap::getFieldAtCoordinate(const Point2D& p) const
{
    auto halfFieldWidth = 0.5 * FieldDimensionsStore::getFieldDimensions().getWidth();
    auto halfFieldLength = 0.5 * FieldDimensionsStore::getFieldDimensions().getLength();

    auto i = floor((p.x + halfFieldWidth) / HeightMapValues::RESOLUTION);
    auto j = floor((p.y + halfFieldLength) / HeightMapValues::RESOLUTION);

    try
    {
        return _heightMap(i,j);
    }
    catch (std::exception&)
    {
        throw std::runtime_error("Coordinate (" + std::to_string(p.x) + "," + std::to_string(p.y) + ") out of bounds");
    }
}

AbstractHeightMap AbstractHeightMap::scale(const float factor) const
{
    TRACE_FUNCTION("");

    AbstractHeightMap result;

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            result._heightMap(i, j).setValue(factor * (this->_heightMap(i, j).getValue()));
        }
    }

    return result;
}

AbstractHeightMap AbstractHeightMap::operator+(const AbstractHeightMap& that) const
{
    AbstractHeightMap sum;

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            sum._heightMap(i, j).setValue((this->_heightMap(i, j).getValue()) + (that._heightMap(i, j).getValue()));
        }
    }

    return sum;
}

void valueToVec3b (const float value, cv::Vec3b& bgr)
{
    /* This function may look expensive. We have done experiments implementing
     * it using a lookup-table. This did not gain us any speed. */

    if ((-100.0 <= value) && (value < -50.0))
    {
        bgr[0] = UCHAR_MAX;
        bgr[1] = cv::saturate_cast<uchar>(floor(5.1 * (value + 100.0)));
        bgr[2] = 0;
    }
    else if ((-50.0 <= value) && (value < 0.0))
    {
        bgr[0] = cv::saturate_cast<uchar>(floor(-5.1 * value));
        bgr[1] = UCHAR_MAX;
        bgr[2] = 0;
    }
    else if ((0.0 <= value) && (value < 50.0))
    {
        bgr[0] = 0;
        bgr[1] = UCHAR_MAX;
        bgr[2] = cv::saturate_cast<uchar>(floor(5.1 * value));
    }
    else if ((50.0 <= value) && (value <= 100.0))
    {
        bgr[0] = 0;
        bgr[1] = cv::saturate_cast<uchar>(floor(-5.1 * (value - 100.0)));
        bgr[2] = UCHAR_MAX;
    }
    else
    {
        throw std::runtime_error("Value out of bounds: " + std::to_string(value));
    }
}

cv::Mat AbstractHeightMap::generateOpenCVMatrix () const
{
    TRACE("Start generating OpenCV Matrix");

    cv::Mat image(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY(), CV_8UC3);
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            valueToVec3b(_heightMap(i, j).getValue(), image.at<cv::Vec3b>(i, j));
        }
    }

    TRACE("Done generating OpenCV Matrix");

    return image;
}

double AbstractHeightMap::angleOnPath (const Point2D& source, const Point2D& at, const Point2D& destination) const
{
    auto angle1 = angle_between_two_points_0_2pi(at.x, at.y, source.x, source.y);
    auto angle2 = angle_between_two_points_0_2pi(at.x, at.y, destination.x, destination.y);

    auto largestAngle = angle1;
    auto smallestAngle = angle2;
    if (angle1 < angle2)
    {
        largestAngle = angle2;
        smallestAngle = angle1;
    }

    auto option1 = largestAngle - smallestAngle;
    auto option2 = 2 * M_PI - largestAngle + smallestAngle;

    if (option1 < option2)
    {
        return option1;
    }
    else
    {
        return option2;
    }
}
