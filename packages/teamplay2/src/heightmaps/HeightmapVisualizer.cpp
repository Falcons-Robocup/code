// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapVisualizer.cpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Coen Tempelaars
 */


#include <cstdlib>


#include "ext/HeightmapVisualizer.hpp"

#include "int/adapters/RTDBInputAdapter.hpp"
#include "int/stores/HeightMapStore.hpp"
#include "int/stores/ConfigurationStore.hpp"

HeightmapVisualizer::HeightmapVisualizer() : _robotID(0), _mtp(nullptr)
{
}

void HeightmapVisualizer::updateRobotId(const int robotID)
{
    // Keep in mind that instantiating MTP adapter is expensive
    if (_robotID != robotID)
    {
        if (_mtp)
        {
            delete _mtp;
        }

        _mtp = new MixedTeamProtocolAdapter(createPlayerId(robotID));
        tpRTDBInputAdapter::getInstance().setMTP(_mtp);
        tpRTDBInputAdapter::getInstance().setWorldModelRobotId(robotID);
        _robotID = robotID;
    }
}

cv::Mat HeightmapVisualizer::visualizeHeightmap (const CompositeHeightmapName& name, const int robotID)
{
    // Init teamplay2 before querying data
    updateRobotId(robotID);

    teamplay::ConfigurationStore::getInstance().getConfiguration().update();
    teamplay::HeightMapStore::getInstance().resetHeightmapPrecalculations();
    tpRTDBInputAdapter::getInstance().getWorldModelData(robotID);
    return teamplay::HeightMapStore::getInstance().generateOpenCVMatrix(name);
}

Point2D HeightmapVisualizer::getOptimum(const CompositeHeightmapName& name, const int robotID)
{
    // Init teamplay2 before querying data
    updateRobotId(robotID);

    teamplay::ConfigurationStore::getInstance().getConfiguration().update();
    teamplay::HeightMapStore::getInstance().resetHeightmapPrecalculations();
    tpRTDBInputAdapter::getInstance().getWorldModelData(robotID);
    return teamplay::HeightMapStore::getInstance().getOptimum(name);
}