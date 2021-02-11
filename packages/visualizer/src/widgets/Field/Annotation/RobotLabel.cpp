// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotLabel.cpp
 *
 *  Created on: November 26, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/Field/Annotation/RobotLabel.h"


void RobotLabel::initialize(int robotID, vtkRenderer *renderer, RobotVisualization* anchor) 
{
    Annotation::initialize(anchor);

    addLabel(robotID, renderer);
}

void RobotLabel::onAnchorPositionChanged(PositionVelocity& posvel)
{
    this->SetPosition(posvel.x, posvel.y, posvel.z); 
}

void RobotLabel::onAnchorVisibilityChanged(bool visible)
{
    this->SetVisibility(visible);
};

void RobotLabel::addLabel(int robotID, vtkRenderer* renderer)
{
    std::ostringstream str;
    str << robotID;

    vtkSmartPointer<vtkVectorText> labelText = vtkSmartPointer<vtkVectorText>::New();
    labelText->SetText(str.str().c_str());
    labelText->Update();

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Scale(0.5, 0.5, 0.5);

    vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    transformFilter->SetInputConnection(labelText->GetOutputPort());
    transformFilter->SetTransform(transform);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    this->SetMapper(mapper);
    this->GetProperty()->SetColor(1, 1, 1);
    this->SetCamera(renderer->GetActiveCamera());
}
