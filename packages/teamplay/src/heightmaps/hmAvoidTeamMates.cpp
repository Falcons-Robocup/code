 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmAvoidTeamMates.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmAvoidTeamMates.hpp"

#include "FalconsCommon.h"
#include "int/stores/robotStore.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


const static float LOW  = 7.0;
const static float HIGH = 13.25;

hmAvoidTeamMates::hmAvoidTeamMates() { }

hmAvoidTeamMates::~hmAvoidTeamMates() { }

void hmAvoidTeamMates::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto teammembers = robotStore::getInstance().getAllRobotsExclOwnRobot();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            double minDistance = std::numeric_limits<double>::max();
            for (auto teammember = teammembers.begin(); teammember != teammembers.end(); teammember++)
            {
                auto distance = calc_distance(teammember->getLocation(), _heightMap(i, j)._center);

                if (distance < minDistance)
                {
                    minDistance = distance;
                }
            }
            _heightMap(i, j).setValue(alpha * (minDistance - low), heightMapValues::MIN, heightMapValues::NEUTRAL);
        }
    }

    return;
}

std::string hmAvoidTeamMates::getDescription() const
{
    return "Avoid team mates";
}

std::string hmAvoidTeamMates::getFilename() const
{
    return "hmAvoidTeamMates";
}
