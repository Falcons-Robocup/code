 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmAvoidBall.cpp
 *
 *  Created on: Jun 05, 2018
 *      Author: Ivo Matthijssen
 */

#include "int/heightmaps/hmAvoidBall.hpp"

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "tracing.hpp"

using namespace teamplay;

const static float MIN_DISTANCE_FROM_BALL = 3.0; //change hardcoded 3.0 into input parameter via action in trees

hmAvoidBall::hmAvoidBall() { }

hmAvoidBall::~hmAvoidBall() { }

void hmAvoidBall::precalculate()
{
    // get ball location
    auto ballPos = ballStore::getBall().getLocation();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto distanceToBall = calc_distance(ballPos, _heightMap(i, j)._center);

            if (distanceToBall < MIN_DISTANCE_FROM_BALL)
            {
            	_heightMap(i, j).setValue(heightMapValues::MIN);
            }
            else
            {
            	_heightMap(i, j).setValue(heightMapValues::NEUTRAL);
            }
        }
    }

    return;
}

std::string hmAvoidBall::getDescription() const
{
    return "Avoid ball";
}

std::string hmAvoidBall::getFilename() const
{
    return "hmAvoidBall";
}
