 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmCloseToBallClaimedLocation.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmCloseToBallClaimedLocation.hpp"

#include "FalconsCommon.h"
#include "int/stores/ballStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


hmCloseToBallClaimedLocation::hmCloseToBallClaimedLocation() { }

hmCloseToBallClaimedLocation::~hmCloseToBallClaimedLocation() { }

void hmCloseToBallClaimedLocation::precalculate()
{
    if (robotStore::getInstance().getOwnRobot().hasBall())
    {
        auto ballClaimedLocation = ballStore::getBall().getClaimedLocation();

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                auto distanceToBallClaimedLocation = calc_distance(ballClaimedLocation, _heightMap(i, j)._center);

                if (distanceToBallClaimedLocation > 3.0)  // TODO: this range should be read from teamplayRules.yaml
                {
                    _heightMap(i, j).setValue(heightMapValues::MIN);
                }
            }
        }
    }

    return;
}

std::string hmCloseToBallClaimedLocation::getDescription() const
{
    return "Close to ball claimed location";
}

std::string hmCloseToBallClaimedLocation::getFilename() const
{
    return "hmCloseToBallClaimedLocation";
}
