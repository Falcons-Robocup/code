// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlannedPath.cpp
 *
 *  Created on: November 26, 2016
 *      Author: Diana Koenraadt
 */

#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>

#include "tracing.hpp"

// Internal:
#include "int/widgets/Field/Annotation/PlannedPath.h"

void PlannedPath::initialize(RobotVisualization* anchor)
{
    _robotVisualization = anchor;
    Annotation::initialize(anchor);
    connect(anchor, SIGNAL(signalPlannedPathChanged(std::vector<PositionVelocity>&)), this, SLOT(onAnchorPlannedPathChanged(std::vector<PositionVelocity>&)));

    this->SetPosition(0, 0, 0);
    _robotVisualization->hideGhosts();
}

void PlannedPath::apply()
{
    TRACE_FUNCTION("");
    TRACE("_actors.size()=%d _path.size()=%d", _actors.size(), _path.size());
    for (size_t i = 0; i < _actors.size(); ++i)
    {
        this->RemovePart(_actors[i]);
    }
    _actors.clear();
    _robotVisualization->hideGhosts();

    if (_path.size() == 0)
    {
        return;
    }

    addActor(_posvel, _path[0]);
    for (size_t i = 0; i < _path.size() - 1; ++i)
    {
        addActor(_path[i], _path[i + 1]);
    }
    for (size_t i = 0; i < _path.size(); ++i)
    {
        _robotVisualization->setGhostPosition((int)i, _path[i]);
    }
}

// Draw targets on path with dashed lines
void PlannedPath::addActor(PositionVelocity& posvel1, PositionVelocity& posvel2)
{
    TRACE_FUNCTION("");
    vtkSmartPointer<vtkLineSource> line1 = vtkSmartPointer<vtkLineSource>::New();
    line1->SetPoint1(posvel1.x, posvel1.y, posvel1.z);
    line1->SetPoint2(posvel2.x, posvel2.y, posvel2.z);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(line1->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(1.5); // TODO: Has no effect. Replace with vtkPoints polygon drawing instead of LineSource needed probably...
    actor->GetProperty()->SetColor(1, 1, 1);
    actor->GetProperty()->SetOpacity(0.5);
    actor->GetProperty()->SetLineStipplePattern(0xf0f0);
    actor->GetProperty()->SetLineStippleRepeatFactor(1);

    _actors.push_back(actor);
    this->AddPart(actor);
}

void PlannedPath::onAnchorPositionChanged(PositionVelocity& posvel)
{
    TRACE_FUNCTION("");
    // Keep annotation at 0,0,0, individual path targets are in world coordinates
    // Store the position for drawing of path
    _posvel = posvel;

    apply();
}

void PlannedPath::onAnchorVisibilityChanged(bool visible)
{
    TRACE_FUNCTION("");
    this->SetVisibility(visible);
}

void PlannedPath::onAnchorPlannedPathChanged(std::vector<PositionVelocity>& path)
{
    TRACE_FUNCTION("");
    _path.clear();
    for (size_t i = 0; i < path.size(); ++i)
    {
        _path.push_back(path[i]);
    }

    apply();
}
