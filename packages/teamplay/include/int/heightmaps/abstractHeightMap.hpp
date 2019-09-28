 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

#include "vector2d.hpp"

namespace teamplay
{

typedef std::map<std::string, std::string> parameterMap_t;

namespace heightMapValues
{
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

    void generateSVG (const std::string&) const;
    void generateJPG (const std::string&) const;
    Point2D getOptimum() const;
    float getValueAtCoordinate(const Point2D&) const;
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

private:
    /* The resolution is the size (in meters) of a square inside a heightmap. */
    float _resolution;
};

} /* namespace teamplay */

#endif /* ABSTRACTHEIGHTMAP_HPP_ */
