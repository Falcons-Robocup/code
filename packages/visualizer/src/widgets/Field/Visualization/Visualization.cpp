// Copyright 2016-2018 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Visualization.cpp
 *
 *  Created on: November 22nd, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/Field/Visualization/Visualization.h"

// Falcons specific:
#include "vector2d.hpp"
#include "tracing.hpp"

Visualization::Visualization()
{
    // Create pointing arrow
    _arrowSource = vtkSmartPointer<vtkArrowSource>::New();
    _arrowSource->SetShaftRadius(0.05);
    _arrowSource->SetTipRadius(0.1);
    _arrowSource->SetTipLength(0.3);
    _arrow = addAsActor(_arrowSource);
}

PositionVelocity Visualization::getPosition()
{
    return _posvel;
}

void Visualization::setPosition(PositionVelocity& posvel)
{
    this->SetPosition(posvel.x, posvel.y, posvel.z);
    this->SetOrientation(0, 0, 180.0 * posvel.phi / M_PI); // radians to degrees
    _posvel = posvel;
    emit signalPositionChanged(posvel);
}

void Visualization::setPosition(polygon2D& area)
{
	this->SetPosition(0.0, 0.0, 0.0);
    _area = area;

    emit signalPositionChanged(area);
}

void Visualization::setPosition(linepoint2D& line)
{
	this->SetPosition(line.getSourcePoint2D().x, line.getSourcePoint2D().y, 0.1);
    _line = line;

    emit signalPositionChanged(line);
}

void Visualization::VisibilityOn()
{
    vtkAssembly::VisibilityOn();
    emit signalVisibilityChanged(true);
}

void Visualization::VisibilityOff()
{
    vtkAssembly::VisibilityOff();
    emit signalVisibilityChanged(false);
}
