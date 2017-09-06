 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
#include "tracer.hpp"

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
