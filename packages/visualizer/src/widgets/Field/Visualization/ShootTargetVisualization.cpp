// Copyright 2017-2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ShootTargetVisualization.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: Coen Tempelaars
 */

#include <vtkSphereSource.h>

// Internal:
#include "int/widgets/Field/Visualization/ShootTargetVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "vector2d.hpp"

ShootTargetVisualization::ShootTargetVisualization()
{
    // Create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(0.0, 0.0, _BALL_DIAMETER / 2.0);
    sphereSource->SetRadius(_BALL_DIAMETER / 2.0);
    _actor = addAsActor(sphereSource);
    _actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // red
    _arrow->VisibilityOff();
    this->VisibilityOff();
}

void ShootTargetVisualization::setPosition(PositionVelocity& posvel)
{
    // call base implementation
    Visualization::setPosition(posvel);
    this->VisibilityOn();
}

void ShootTargetVisualization::visibilityOff()
{
    this->VisibilityOff();
}
