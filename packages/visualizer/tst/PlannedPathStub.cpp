// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlannedPathStub.cpp
 *
 *  Created on: December 10, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "PlannedPathStub.h"

void PlannedPathStub::initialize(RobotVisualization* anchor)
{
    connect(anchor, SIGNAL(signalPlannedPathChanged(std::vector<PositionVelocity>&)), this, SLOT(onSetPath(std::vector<PositionVelocity>&)));
}

void PlannedPathStub::onSetPath(std::vector<PositionVelocity>& path)
{
    onPathChangedCalled = true;
    _path.clear();
    for (int i = 0; i < (int)path.size(); ++i)
    {
        _path.push_back(path[i]);
    }
}
