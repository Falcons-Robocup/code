// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotLabel.h
 *
 *  Created on: November 26, 2016
 *      Author: Diana Koenraadt
 */

#include <vtkVectorText.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <math.h> // sin, cos
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <stdlib.h> // rand

// Internal:
#include "int/widgets/Field/Annotation/CollisionBalloon.h"

void CollisionBalloon::initialize(RobotVisualization* anchor, CollisionBalloonDimensions dim) 
{
    Annotation::initialize(anchor);

    addBalloon(dim);

    // Set position only once.
    PositionVelocity posvel = anchor->getPosition();
    this->SetPosition(posvel.x, posvel.y, posvel.z);
}

void CollisionBalloon::setColor(double red, double green, double blue)
{
    _actor->GetProperty()->SetColor(red, green, blue);
}

void CollisionBalloon::onAnchorVisibilityChanged(bool visible)
{
    this->SetVisibility(visible);
};

void CollisionBalloon::addBalloon(CollisionBalloonDimensions dim)
{
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();

    vtkSmartPointer<vtkPoints> innerPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> outerPoints = vtkSmartPointer<vtkPoints>::New();
    for (double i = 0; i < 360; i += 20)
    {
        double x = dim.InnerRadiusX * cos(i * M_PI / 180);
        double y = dim.InnerRadiusY * sin(i * M_PI / 180);
        innerPoints->InsertNextPoint(x, y, 0);

        x = dim.OuterRadiusX * cos(i * M_PI / 180);
        y = dim.OuterRadiusY * sin(i * M_PI / 180);
        outerPoints->InsertNextPoint(x, y, 0);
    }
    vtkSmartPointer<vtkPolygon> innerPolygon = vtkSmartPointer<vtkPolygon>::New();
    innerPolygon->GetPointIds()->SetNumberOfIds(innerPoints->GetNumberOfPoints());
    for (int i = 0; i < innerPoints->GetNumberOfPoints(); i++)
    {
        innerPolygon->GetPointIds()->SetId(i /* index */, i /* point id */); // Iterate over points in the order they were created.
    }
    polygons->InsertNextCell(innerPolygon);

    for (int i = 0; i < innerPoints->GetNumberOfPoints(); ++i)
    {
        vtkSmartPointer<vtkPolygon> triangle = vtkSmartPointer<vtkPolygon>::New();
        triangle->GetPointIds()->SetNumberOfIds(3);

        if (i == innerPoints->GetNumberOfPoints() - 1)
        {
            // Last triangle should connect first and last points
            triangle->GetPointIds()->SetId(0, i);
            triangle->GetPointIds()->SetId(1, innerPoints->GetNumberOfPoints() + i); // outer points will be appended to innerPoints, count onwards
            triangle->GetPointIds()->SetId(2, 0);
        }
        else
        {
            triangle->GetPointIds()->SetId(0, i);
            triangle->GetPointIds()->SetId(1, innerPoints->GetNumberOfPoints() + i); // outer points will be appended to innerPoints, count onwards
            triangle->GetPointIds()->SetId(2, i + 1);
        }

        polygons->InsertNextCell(triangle);
    }

    for (int i = 0; i < outerPoints->GetNumberOfPoints(); ++i)
    {
        innerPoints->InsertNextPoint(outerPoints->GetPoint(i));
    }
    
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(innerPoints);
    polyData->SetPolys(polygons);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    _actor = vtkSmartPointer<vtkActor>::New();
    _actor->SetMapper(mapper);
    _actor->GetProperty()->SetColor(1, 0, 0);

    this->AddPart(_actor);
}
