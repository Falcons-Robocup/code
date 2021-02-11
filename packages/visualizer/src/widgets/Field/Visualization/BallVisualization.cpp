// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Diana Koenraadt, October 2016
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
#include "int/widgets/Field/Visualization/BallVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "vector2d.hpp"

BallVisualization::BallVisualization()
{
    // Create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(0.0, 0.0, _BALL_DIAMETER / 2.0);
    sphereSource->SetRadius(_BALL_DIAMETER / 2.0);
    _actor = addAsActor(sphereSource);
    _actor->GetProperty()->SetColor(1.0, 1.0, 1.0);

    // Create pointing arrow
    _arrow->GetProperty()->SetColor(1.0, 1.0, 1.0);
    _arrow->SetPosition(0, 0, _BALL_DIAMETER / 2.0);

    // Create an assembly consisting of the various parts
    this->VisibilityOn();
}

double BallVisualization::getDiameter()
{
    return _BALL_DIAMETER;
}

void BallVisualization::setPosition(PositionVelocity& posvel)
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

void BallVisualization::setColor(BallColor color, float redFactor)
{
    // input clipping
    redFactor = std::max(redFactor, 0.0f);
    redFactor = std::min(redFactor, 1.0f);
    // set base color
    float red = 1.0;
    float green = 1.0;
    float blue = 0.0;
    if (color == CYAN)
    {
        red = 0.0;
        blue = 1.0;
    }
    // factor in quality as red
    blue *= (1.0 - redFactor);
    green *= (1.0 - redFactor);
    // modify existing actors
    if (_actor != NULL)
    {
        _actor->GetProperty()->SetColor(red, green, blue);
    }
    if (_arrow != NULL)
    {
        _arrow->GetProperty()->SetColor(red, green, blue);
    }
}

void BallVisualization::setOpacity(float opacity)
{
    // input clipping
    opacity = std::max(opacity, 0.1f);
    opacity = std::min(opacity, 1.0f);
    // modify existing actors
    if (_actor != NULL)
    {
        _actor->GetProperty()->SetOpacity(opacity);
    }
    if (_arrow != NULL)
    {
        _actor->GetProperty()->SetOpacity(opacity);
    }
}


