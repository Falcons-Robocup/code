// Copyright 2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapVisualizer.cpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Coen Tempelaars
 */

#include "ext/heightmapVisualizer.hpp"

#include <cstdlib>

#include "int/adapters/ConfigAdapter.hpp"
#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/stores/heightMapStore.hpp"

HeightmapVisualizer::HeightmapVisualizer()
{
    auto configAdapter = new ConfigAdapter();
    configAdapter->loadYAML(determineConfig("teamplay"));
}

cv::Mat HeightmapVisualizer::visualizeHeightmap (const CompositeHeightmapName& name, const int robotID)
{
    /* Obtain worldmodel data for given robot, precalculate all heightmaps and return an OpenCV image of given composite heightmap */
    cRTDBInputAdapter::getInstance().getWorldModelData(robotID);
    teamplay::heightMapStore::getInstance().precalculateAll();
    return teamplay::heightMapStore::getInstance().generateOpenCVMatrix(name);
}
