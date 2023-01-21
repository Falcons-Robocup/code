// Copyright 2016-2022 Diana Koenraadt (Falcons)
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

    _labelText = vtkSmartPointer<vtkVectorText>::New();
    _labelText->SetText(str.str().c_str());
    _labelText->Update();

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Scale(0.5, 0.5, 0.5);

    vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    transformFilter->SetInputConnection(_labelText->GetOutputPort());
    transformFilter->SetTransform(transform);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    untrack();
    this->SetMapper(mapper);
    this->SetCamera(renderer->GetActiveCamera());
}

void RobotLabel::setRole(int robotId, std::string text)
{
    std::ostringstream str;
    str << robotId << ": ";

    // Convert the full role name into an abbreviation that doesn't take up half the screen
    // We do this by simply taking the first character of each word
    // e.g.
    //   GOALKEEPER -> G
    //   ATTACKER_MAIN -> AM
    size_t pos = 0;
    str << text.substr(0, 1);
    while ((pos = text.find("_")) != std::string::npos)
    {
        text.erase(0, pos + 1);
        str << text.substr(0, 1);
    }

    _labelText->SetText(str.str().c_str());
    _labelText->Update();
}

void RobotLabel::track()
{
    this->GetProperty()->SetColor(1, 0, 0);
}

void RobotLabel::untrack()
{
    this->GetProperty()->SetColor(1, 1, 1);
}
