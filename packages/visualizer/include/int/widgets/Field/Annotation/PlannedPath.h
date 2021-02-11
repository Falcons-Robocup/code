// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlannedPath.h
 *
 *  Created on: December 7th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef PLANNEDPATH_H
#define PLANNEDPATH_H

#include <vtkAssembly.h>
#include <vtkProp3D.h>
#include <sstream>

// Internal:
#include "int/widgets/Field/Visualization/RobotVisualization.h"
#include "Annotation.h"

/*
* Class that plots the planned path the robot is going to take in a dashed line
*/
class PlannedPath : public Annotation, public vtkAssembly
{
    Q_OBJECT

public:
    static PlannedPath* New()
    {
        return new PlannedPath();
    }

    void initialize(RobotVisualization* anchor);

private:
    RobotVisualization* _robotVisualization;
    PositionVelocity _posvel;
    std::vector<PositionVelocity> _path;
    std::vector<vtkSmartPointer<vtkActor>> _actors;

    void apply();
    void addActor(PositionVelocity& posvel1, PositionVelocity& posvel2);

public Q_SLOTS:
    virtual void onAnchorPositionChanged(PositionVelocity& posvel) override;
    virtual void onAnchorVisibilityChanged(bool visible) override;

    void onAnchorPlannedPathChanged(std::vector<PositionVelocity>& path);
};

#endif // PLANNEDPATH_H
