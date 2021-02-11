// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlannedPathStub.h
 *
 *  Created on: December 10th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef PLANNEDPATHSTUB_H
#define PLANNEDPATHSTUB_H

// Internal:
#include "int/widgets/Field/Annotation/Annotation.h"
#include "int/widgets/Field/Visualization/RobotVisualization.h"

class PlannedPathStub : public Annotation
{
    Q_OBJECT

public:
    static PlannedPathStub* New()
    {
        return new PlannedPathStub();
    }

    void initialize(RobotVisualization* anchor);
    bool onPathChangedCalled = false;
    std::vector<PositionVelocity> _path;

public Q_SLOTS:
    void onSetPath(std::vector<PositionVelocity>& path);
};

#endif // PLANNEDPATHSTUB_H
