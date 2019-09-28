 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmObstaclesBlockingOppGoal.cpp
 *
 *  Created on: Jan 6, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmObstaclesBlockingOppGoal.hpp"

#include <cmath>
#include <limits>

#include "FalconsCommon.h"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


const static float LOW  = 0.417;
const static float HIGH = 0.734;

const static float DISTANCE_BEHIND_OBSTACLE = 1.0;

hmObstaclesBlockingOppGoal::hmObstaclesBlockingOppGoal() { }

hmObstaclesBlockingOppGoal::~hmObstaclesBlockingOppGoal() { }

void hmObstaclesBlockingOppGoal::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto oppGoal = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALLINE_CENTER);
    auto obstacles = obstacleStore::getInstance().getAllObstacles();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            double minDifference = std::numeric_limits<double>::max();
            for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++)
            {
                auto obstacleLocation = obstacle->getLocation();
                auto refinedObstacleLocation = (1 - (DISTANCE_BEHIND_OBSTACLE / (obstacleLocation - oppGoal).size())) * (obstacleLocation - oppGoal) + oppGoal;

                auto oppToGoalAngle = angle_between_two_points_0_2pi (refinedObstacleLocation.x, refinedObstacleLocation.y, oppGoal.x, oppGoal.y);
                auto heightMapFieldToOppAngle = angle_between_two_points_0_2pi (_heightMap(i, j)._center.x, _heightMap(i, j)._center.y, refinedObstacleLocation.x, refinedObstacleLocation.y);
                auto absDifference = std::abs(oppToGoalAngle - heightMapFieldToOppAngle);

                if (absDifference < minDifference)
                {
                    minDifference = absDifference;
                }
            }
            _heightMap(i, j).setValue(alpha * (minDifference - low), heightMapValues::MIN, heightMapValues::NEUTRAL);
        }
    }

    return;
}

std::string hmObstaclesBlockingOppGoal::getDescription() const
{
    return "Avoid obstacles that block the opponent goal";
}

std::string hmObstaclesBlockingOppGoal::getFilename() const
{
    return "hmObstaclesBlockingOppGoal";
}
