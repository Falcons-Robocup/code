 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmObstaclesBlockingBall.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmObstaclesBlockingBall.hpp"

#include <cmath>
#include <limits>

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 2.8;
const static float HIGH = 2.6;

hmObstaclesBlockingBall::hmObstaclesBlockingBall() { }

hmObstaclesBlockingBall::~hmObstaclesBlockingBall() { }

void hmObstaclesBlockingBall::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto obstacles = obstacleStore::getInstance().getAllObstacles();
    auto ballLocation = ballStore::getBall().getLocation();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            double minValue = std::numeric_limits<double>::max();
            for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++)
            {
                auto obstacleLocation = obstacle->getLocation();
                auto angle = angleOnPath(_heightMap(i, j)._center, obstacleLocation, ballLocation);
                auto value = alpha * (angle - low);

                if (value < minValue)
                {
                    minValue = value;
                }
            }
            _heightMap(i, j).setValue(minValue, heightMapValues::MIN, heightMapValues::NEUTRAL);
        }
    }

    return;
}

std::string hmObstaclesBlockingBall::getDescription() const
{
    return "Avoid obstacles that block the ball";
}

std::string hmObstaclesBlockingBall::getFilename() const
{
    return "hmObstaclesBlockingBall";
}
