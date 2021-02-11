// Copyright 2017-2018 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VisionLock.cpp
 *
 *  Created on: December 18th, 2016
 *      Author: Diana Koenraadt
 */

#include <vtkSmartPointer.h>
#include <vtkPNGReader.h>
#include <vtkImageMapper.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>

#include <QFile>

// Internal:
#include "int/widgets/Field/Annotation/VisionLock.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"

void VisionLock::initialize(RobotVisualization* anchor) 
{
    Annotation::initialize(anchor);

    addIcon(anchor->getWidth(), anchor->getHeight());
}

void VisionLock::onAnchorPositionChanged(PositionVelocity& posvel)
{
    this->SetPosition(posvel.x, posvel.y, posvel.z);
}

void VisionLock::onAnchorVisibilityChanged(bool visible)
{
    this->SetVisibility(visible);
}

void VisionLock::addIcon(double width, double height)
{
    QFile::copy(":/eye.png", "eye.png"); // extract the .stl resource from the application for use by vtk
    QFile::copy(":/eye_strikeout.png", "eye_strikeout.png"); // extract the .stl resource from the application for use by vtk

    vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();

    if (!reader->CanReadFile("eye.png"))
    {
        TRACE("Could not read eye.png for vision lock annotation.");
    }
    if (!reader->CanReadFile("eye_strikeout.png"))
    {
        TRACE("Could not read eye_strikeout.png for vision lock annotation.");
    }

    reader->SetFileName("eye.png");
    reader->Update();

    vtkSmartPointer<vtkImageActor> actor = vtkSmartPointer<vtkImageActor>::New();

    vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New();
    vtkImageData* output = reader->GetOutput();
    data->ShallowCopy(output);

    int *dims = data->GetDimensions();
    data->SetDimensions(dims[0], dims[1], 1); // set dimensions in each axis
    data->SetSpacing(width / dims[0], height / dims[1], 0.1); // sets the size of a voxel, normalizes the image to the same dimensions as the robot.
    data->SetOrigin(0, 0, 0);

    actor->SetInput(data);

    this->AddPart(actor);
}
