// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmCloseToBallClaimedLocation.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmCloseToBallClaimedLocation.hpp"

#include "falconsCommon.hpp"
#include "int/stores/BallStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


hmCloseToBallClaimedLocation::hmCloseToBallClaimedLocation() { }

hmCloseToBallClaimedLocation::~hmCloseToBallClaimedLocation() { }

void hmCloseToBallClaimedLocation::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        if (RobotStore::getInstance().getOwnRobot().hasBall())
        {
            auto ballClaimedLocation = BallStore::getBall().getClaimedLocation();

            for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
            {
                for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
                {
                    auto distanceToBallClaimedLocation = calc_distance(ballClaimedLocation, _heightMap(i, j)._center);

                    if (distanceToBallClaimedLocation > 3.0) // TODO: this range should be read from teamplayRules.yaml
                    {
                        _heightMap(i, j).setValue(HeightMapValues::MIN);
                    }
                    else
                    {
                        _heightMap(i, j).setValue(HeightMapValues::NEUTRAL);
                    }
                }
            }
        }
        _heightmapPrecalculated = true;
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
