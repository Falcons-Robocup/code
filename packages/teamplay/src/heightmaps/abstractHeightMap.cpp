 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * abstractHeightMap.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/abstractHeightMap.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "boost/format.hpp"
#include <opencv2/opencv.hpp>

#include "FalconsCommon.h"
#include "tracing.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


const static int fieldsize = 10;

void heightMapField::setValue (const float value)
{
    setValue(value, heightMapValues::MIN, heightMapValues::MAX);
}

void heightMapField::setValue (const float value, const float floor, const float ceil)
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

float heightMapField::getValue() const
{
    return _value;
}

abstractHeightMap::abstractHeightMap()
{
    _resolution = 0.25;
    reset();
}

abstractHeightMap::~abstractHeightMap() { }

unsigned int abstractHeightMap::getNrOfHeightMapFieldsInX() const
{
    return _heightMap.size1();
}

unsigned int abstractHeightMap::getNrOfHeightMapFieldsInY() const
{
    return _heightMap.size2();
}

void abstractHeightMap::reset()
{
    _heightMap.resize((int)ceil(fieldDimensionsStore::getFieldDimensions().getWidth() / _resolution),
                    (int)ceil(fieldDimensionsStore::getFieldDimensions().getLength() / _resolution));

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto center = Point2D( (i - floor(getNrOfHeightMapFieldsInX() / 2)) * _resolution,
                                   (j - floor(getNrOfHeightMapFieldsInY() / 2)) * _resolution);
            _heightMap(i, j)._center = center;
            _heightMap(i, j).setValue(heightMapValues::NEUTRAL);
        }
    }
}

void abstractHeightMap::precalculate()
{
    throw std::runtime_error("Not implemented");
}

abstractHeightMap abstractHeightMap::refine(const parameterMap_t& params)
{
    return *this;
}

std::string abstractHeightMap::getDescription() const
{
    throw std::runtime_error("Not implemented");
}

std::string abstractHeightMap::getFilename() const
{
    throw std::runtime_error("Not implemented");
}

Point2D abstractHeightMap::getOptimum() const
{
    heightMapField optimum;
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

    return optimum._center;
}

float abstractHeightMap::getValueAtCoordinate(const Point2D& p) const
{
    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            if (calc_distance(_heightMap(i, j)._center, p) < _resolution)
            {
                return _heightMap(i, j).getValue();
            }
        }
    }
    throw std::runtime_error("Coordinate (" + std::to_string(p.x) + "," + std::to_string(p.y) + ") out of bounds");
}

abstractHeightMap abstractHeightMap::scale(const float factor) const
{
    abstractHeightMap result;

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            result._heightMap(i, j).setValue(factor * (this->_heightMap(i, j).getValue()));
        }
    }

    return result;
}

abstractHeightMap abstractHeightMap::operator+(const abstractHeightMap& that) const
{
    abstractHeightMap sum;

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            sum._heightMap(i, j).setValue((this->_heightMap(i, j).getValue()) + (that._heightMap(i, j).getValue()));
        }
    }

    return sum;
}

std::string valueToSVGColor (const float value)
{
    /* This function may look expensive. We have done experiments implementing
     * it using a lookup-table. This did not gain us any speed. */

    int r, g, b = 0;

    if ((-100.0 <= value) && (value < -50.0))
    {
        b = 255;
        g = (int)floor(5.1 * (value + 100.0));
        r = 0;
    }
    else if ((-50.0 <= value) && (value < 0.0))
    {
        b = (int)floor(-5.1 * value);
        g = 255;
        r = 0;
    }
    else if ((0.0 <= value) && (value < 50.0))
    {
        b = 0;
        g = 255;
        r = (int)floor(5.1 * value);
    }
    else if ((50.0 <= value) && (value <= 100.0))
    {
        b = 0;
        g = (int)floor(-5.1 * (value - 100.0));
        r = 255;
    }
    else
    {
        throw std::runtime_error("Value out of bounds: " + std::to_string(value));
    }

    return str(boost::format("fill:rgb(%1%,%2%,%3%)") % r % g % b);
}

void abstractHeightMap::generateSVG (const std::string& filename) const
{
    /* Warning: this function is expensive. Consider using generateJPG instead. */

    std::ofstream file (filename);
    file << "<!DOCTYPE html>\n<html>\n<body>\n";

    file << boost::format("<svg width='%1%' height='%2%'>\n")
            % (fieldsize * getNrOfHeightMapFieldsInY())
            % (fieldsize * getNrOfHeightMapFieldsInX());

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            try
            {
                file << boost::format("<rect width='%1%' height='%1%' x='%2%' y='%3%' style='%4%' />\n")
                        % fieldsize % (10 * j) % (10 * i)
                        % valueToSVGColor(_heightMap(i, j).getValue());
            }
            catch (std::exception& e)
            {
                throw std::runtime_error("Error converting heightmap field " + std::to_string(i) + ","
                                         + std::to_string(j) + " to color: " + e.what());
            }
        }
    }

    file << "</svg>\n";

    file << "</body>\n</html>\n";
    file.close();
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

void abstractHeightMap::generateJPG (const std::string& filename) const
{
    /* This function is implemented using opencv. On modern hardware,
     * it is a factor 6 cheaper than generateSVG */

    TRACE("Start generating jpg: ") << filename;

    cv::Mat image(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY(), CV_8UC3);
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            valueToVec3b(_heightMap(i, j).getValue(), image.at<cv::Vec3b>(i, j));
        }
    }

    cv::Mat scaledImage(fieldsize * getNrOfHeightMapFieldsInX(), fieldsize * getNrOfHeightMapFieldsInY(), CV_8UC3);
    cv::resize(image, scaledImage, scaledImage.size());

    imwrite("/var/tmp/" + filename + ".jpg", scaledImage);

    TRACE("Done generating jpg: ") << filename;
}

double abstractHeightMap::angleOnPath (const Point2D& source, const Point2D& at, const Point2D& destination) const
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
