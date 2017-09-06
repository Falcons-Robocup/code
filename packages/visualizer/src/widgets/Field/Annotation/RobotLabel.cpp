 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
