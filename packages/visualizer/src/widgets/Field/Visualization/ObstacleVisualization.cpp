// Copyright 2017-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Jan Feitsma, March 2017
 */

#include <vtkAlgorithm.h>
#include <vtkAlgorithmOutput.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkObjectFactory.h>

// Internal:
#include "int/widgets/Field/Visualization/ObstacleVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "vector2d.hpp"

ObstacleVisualization::ObstacleVisualization()
{
    // Create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(0.0, 0.0, _OBSTACLE_DIAMETER / 2.0);
    sphereSource->SetRadius(_OBSTACLE_DIAMETER / 2.0);
    vtkSmartPointer<vtkActor> actor = addAsActor(sphereSource);
    actor->GetProperty()->SetColor(0.5, 0.0, 0.0); // uniform dark-red coloring
    actor->GetProperty()->SetOpacity(_OBSTACLE_OPACITY);
    
    // Create pointing arrow
    _arrow->GetProperty()->SetColor(0.5, 0.0, 0.0); // uniform dark-red coloring
    _arrow->GetProperty()->SetOpacity(_OBSTACLE_OPACITY);
    _arrow->SetPosition(0, 0, _OBSTACLE_DIAMETER / 2.0);

    // Create an assembly consisting of the various parts
    this->VisibilityOn();
}

double ObstacleVisualization::getDiameter()
{
    return _OBSTACLE_DIAMETER;
}

void ObstacleVisualization::setPosition(PositionVelocity& posvel)
{
    // velocity vector
    Vector2D speed(posvel.vx, posvel.vy);
    double length = vectorsize(speed);
    double orientation = atan2(posvel.vy, posvel.vx);
    _arrow->SetOrientation(0, 0, 180.0 * orientation / M_PI); // radians to degrees
    if (length > 0.1)
    {
        double arrowScaling = 0.3;
        double scale[3] = { length * arrowScaling, 1, 1 };
        _arrow->SetScale(scale);
        _arrowSource->SetTipLength(0.3 / length);
        _arrow->VisibilityOn();
    }
    else
    {
        _arrow->VisibilityOff();
    }

    // call base implementation
    Visualization::setPosition(posvel);
}

