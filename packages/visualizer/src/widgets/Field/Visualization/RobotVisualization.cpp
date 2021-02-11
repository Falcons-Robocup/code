// Copyright 2016-2020 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Jan Feitsma, July 2016
 *
 * Aggregate a group of actors and actors relative to it
 * so we can hide/highlight them as one; link together data, etc.
 */

#include <vtkTextActor.h>
#include <vtkAlgorithm.h>
#include <vtkAlgorithmOutput.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkSTLReader.h>
#include <QApplication>
#include <QFile>

// Internal:
#include "int/widgets/Field/Visualization/RobotVisualization.h"
#include "int/widgets/Field/Annotation/PlannedPath.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"

void RobotVisualization::initialize(int robotID, vtkRenderer *renderer)
{
    _robotID = robotID;
    std::string stlFile = pathToCodeRepo() + "/packages/visualizer/rc/robot.stl";
    QFile file(stlFile.c_str()); // TODO extract the .stl resource from the application instead of reading it from the source location
    if (file.exists())
    {
        // Load STL
        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(stlFile.c_str());
        reader->Update();
        _actor = addAsActor(reader);
        _actor->GetProperty()->SetColor(0.1, 0.1, 1.0);

        // Add ghost robots for path visualization
        int MAX_PATH_LENGTH = 5;
        _pathGhosts.clear();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(reader->GetOutputPort());
        for (int it = 0; it < MAX_PATH_LENGTH; ++it)
        {
            vtkSmartPointer<vtkActor> ghostActor = vtkSmartPointer<vtkActor>::New();
            ghostActor->GetProperty()->SetColor(179/255.0, 179/255.0, 204/255.0); // grey
            ghostActor->GetProperty()->SetOpacity(0.3);
            ghostActor->SetMapper(mapper);
            renderer->AddActor(ghostActor);
            _pathGhosts.push_back(ghostActor);
        }
    }
    else
    {
        // Fall back on sphere shape if the STL file could not be copied
        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(0.0, 0.0, _BALL_DIAMETER / 2.0);
        sphereSource->SetRadius(_BALL_DIAMETER / 2.0);
        _actor = addAsActor(sphereSource);
        _actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
    }

    // only show arrow when robot has ball, to indicate shooting aim
    _arrow->VisibilityOff();
    _arrow->GetProperty()->SetColor(0.1, 0.1, 1.0);
    _arrow->GetProperty()->SetOpacity(0.3);
    _arrow->SetPosition(0, 0, 0.2);

    // Create annotation for path planning and add to renderer
    vtkSmartPointer<PlannedPath> path = vtkSmartPointer<PlannedPath>::New();
    path->initialize(this);
    renderer->AddActor(path);

    // TODO disable for now.
    // Desired to inherit VisionLock from vtkFollower, but this cannot handle PNG images.
    // Current implementation of VisionLock does not follow the camera.
    //vtkSmartPointer<VisionLock> lock = vtkSmartPointer<VisionLock>::New();
    //lock->initialize(renderer, this);
    //renderer->AddActor(lock);

    _pathPlanningEnabled = true;

    this->VisibilityOff();

    _blinkTimer = new QTimer(this);
    connect(_blinkTimer, SIGNAL(timeout()), this, SLOT(blink()));
}

void RobotVisualization::hideGhosts()
{
    TRACE_FUNCTION("");
    TRACE("_pathGhosts.size=%d", (int)_pathGhosts.size());
    for (int it = 0; it < (int)_pathGhosts.size(); ++it)
    {
        _pathGhosts.at(it)->VisibilityOff();
    }
}

void RobotVisualization::setGhostPosition(int ghostId, PositionVelocity const& posvel)
{
    TRACE_FUNCTION("");
    TRACE("ghostId=%d, _pathGhosts.size=%d", ghostId, (int)_pathGhosts.size());
    if (ghostId < (int)_pathGhosts.size())
    {
        auto ghost = _pathGhosts.at(ghostId);
        ghost->SetPosition(posvel.x, posvel.y, 0.0);
        ghost->SetOrientation(0, 0, 180.0 * posvel.phi / M_PI); // radians to degrees
        TRACE("robotId=%d ghostId=%d pose=(%6.2f, %6.2f, %6.2f)", _robotID, ghostId, posvel.x, posvel.y, posvel.phi);
        ghost->VisibilityOn();
    }
}

void RobotVisualization::hideArrow()
{
    _arrow->VisibilityOff();
}

void RobotVisualization::showArrow()
{
    // use arrow to visualize orientation (aiming direction)
    _arrow->SetOrientation(0, 0, 0);
    double length = 5.0;
    double scale[3] = { length, 1, 1 };
    _arrow->SetScale(scale);
    _arrowSource->SetTipLength(0.3 / length);
    _arrow->VisibilityOn();
}

void RobotVisualization::setPosition(PositionVelocity& posvel)
{
    // call base implementation
    Visualization::setPosition(posvel);
}

void RobotVisualization::setPath(std::vector<PositionVelocity>& path)
{
    if (_pathPlanningEnabled)
    {
        emit signalPlannedPathChanged(path);
    }
}

void RobotVisualization::setPathPlanningEnabled(bool enabled) // Allows enabling/disabling of path processing
{
    if (!enabled)
    {
        // clear the path
        std::vector<PositionVelocity> empty;
        setPath(empty);
    }
    _pathPlanningEnabled = enabled;
}

void RobotVisualization::setStatus(int color)
{
    // TODO: when NOK, enable blinking mode

    if (color == 0 )
    {
        _actor->GetProperty()->SetColor(0, 0, 1); // blue
    }
    else if (color == 1 )
    {
        _actor->GetProperty()->SetColor(1, 0.75, 0); // yellow
    }
    else if (color == 2)
    {
        _actor->GetProperty()->SetColor(1, 0, 0); // red
    }
    else
    {}
}

void RobotVisualization::blinkOn()
{
    if (_blinkTimer != NULL)
    {
        _blinkTimer->start(1000);
    }
}

void RobotVisualization::blinkOff()
{
    if (_blinkTimer != NULL)
    {
        _blinkTimer->stop();

        // Set back to green player color
        _actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
        _arrow->GetProperty()->SetColor(0.0, 1.0, 0.0);
    }
}

void RobotVisualization::blink()
{
    double color[3];
    _actor->GetProperty()->GetColor(color);

    // Blink red / green
    if (color[0] > 0.5)
    {
        _actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
        _arrow->GetProperty()->SetColor(0.0, 1.0, 0.0);
    }
    else
    {
        _actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
        _arrow->GetProperty()->SetColor(1.0, 0.0, 0.0);
    }
}

