 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmNearPosition.hpp
 *
 *  Created on: June 5, 2018
 *      Author: Jan Feitsma
 */

#include <boost/lexical_cast.hpp>

#include "int/heightmaps/hmNearPosition.hpp"

#include "falconsCommon.hpp"

#include "tracing.hpp"

using namespace teamplay;


hmNearPosition::hmNearPosition()
{
    reset();
}

hmNearPosition::~hmNearPosition()
{
}

void hmNearPosition::precalculate()
{
    // cannot do anything yet, need parameters
    return;
}

abstractHeightMap hmNearPosition::refine(const parameterMap_t& params)
{
    // configuration
    float alpha = 10.0; // magic number
    // extract parameters
    float positionX = 0.0;
    float positionY = 0.0;
    auto it = params.find("positionX");
    if (it != params.end())
    {
        positionX = boost::lexical_cast<float>(it->second);
    }
    it = params.find("positionY");
    if (it != params.end())
    {
        positionY = boost::lexical_cast<float>(it->second);
    }
    // calculate the height map
    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto distance = calc_distance(Point2D(positionX, positionY), _heightMap(i, j)._center);
            _heightMap(i, j).setValue(100.0 - alpha * distance);
        }
    }
    return *this;
}

std::string hmNearPosition::getDescription() const
{
    return "Near position";
}

std::string hmNearPosition::getFilename() const
{
    return "hmNearPosition";
}

