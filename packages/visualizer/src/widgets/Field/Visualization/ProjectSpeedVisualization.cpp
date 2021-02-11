// Copyright 2017-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Tim Kouters, July 2017
 */

#include <vtkAlgorithm.h>
#include <vtkAlgorithmOutput.h>
#include <vtkCubeSource.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkObjectFactory.h>

// Internal:
#include "int/widgets/Field/Visualization/projectSpeedVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "linepoint2D.hpp"

projectSpeedVisualization::projectSpeedVisualization() : Visualization()
{
	// Hide
	this->VisibilityOff();
    _arrow->VisibilityOff();

    _lineSource = vtkSmartPointer<vtkLineSource>::New();
	_lineSource->SetPoint1(0.0, 0.0, 0.1);
	_lineSource->SetPoint2(0.0, 0.0, 0.1);
	_lineSource->Update();

    _actor = addAsActor(_lineSource);
    _actor->GetProperty()->SetColor(0.3, 0.0, 1.0);
    _actor->GetProperty()->SetLineWidth(6);
    _actor->SetPosition(0.0, 0.0, 0.0);
    _actor->Modified();
    _actor->VisibilityOff();
}


void projectSpeedVisualization::setPosition(linepoint2D& line)
{
	_actor->SetOrigin(line.getSourcePoint2D().x, line.getSourcePoint2D().y, 0.1);
	Vector2D dst = line.getDestinationVector2D() - line.getSourceVector2D();
	_lineSource->SetPoint1(dst.x, dst.y, 0.1);

	this->VisibilityOn();
	_actor->VisibilityOn();

    // call base implementation
    Visualization::setPosition(line);
}

