 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
    std::string stlFile = "/home/robocup/falcons/code/packages/visualizer/rc/robot.stl";
    QFile file(stlFile.c_str()); // TODO extract the .stl resource from the application instead of reading it from the source location
    if (file.exists())
    {
        // Load STL
        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(stlFile.c_str()); 
        reader->Update();
        _actor = addAsActor(reader);
        _actor->GetProperty()->SetColor(0.1, 0.1, 1.0);
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
    
    // no arrow anymore
    _arrow->VisibilityOff();

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

