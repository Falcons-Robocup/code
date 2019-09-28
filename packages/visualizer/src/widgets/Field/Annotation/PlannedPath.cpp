 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PlannedPath.cpp
 *
 *  Created on: November 26, 2016
 *      Author: Diana Koenraadt
 */

#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>

// Internal:
#include "int/widgets/Field/Annotation/PlannedPath.h"

void PlannedPath::initialize(RobotVisualization* anchor) 
{
    Annotation::initialize(anchor);
    connect(anchor, SIGNAL(signalPlannedPathChanged(std::vector<PositionVelocity>&)), this, SLOT(onAnchorPlannedPathChanged(std::vector<PositionVelocity>&)));

    this->SetPosition(0, 0, 0);
}

void PlannedPath::apply()
{
    for (size_t i = 0; i < _actors.size(); ++i)
    {
        this->RemovePart(_actors[i]);
    }
    _actors.clear();

    if (_path.size() == 0)
    {
        return;
    }

    addActor(_posvel, _path[0]);
    for (size_t i = 0; i < _path.size() - 1; ++i)
    {
        addActor(_path[i], _path[i + 1]);
    }
}

// Draw targets on path with dashed lines
void PlannedPath::addActor(PositionVelocity& posvel1, PositionVelocity& posvel2)
{
    vtkSmartPointer<vtkLineSource> line1 = vtkSmartPointer<vtkLineSource>::New();
    line1->SetPoint1(posvel1.x, posvel1.y, posvel1.z);
    line1->SetPoint2(posvel2.x, posvel2.y, posvel2.z);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(line1->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(1.5); // TODO: Has no effect. Replace with vtkPoints polygon drawing instead of LineSource needed probably...
    actor->GetProperty()->SetColor(1, 1, 1);
    actor->GetProperty()->SetOpacity(0.5);
    actor->GetProperty()->SetLineStipplePattern(0xf0f0);
    actor->GetProperty()->SetLineStippleRepeatFactor(1);

    _actors.push_back(actor);
    this->AddPart(actor);
}

void PlannedPath::onAnchorPositionChanged(PositionVelocity& posvel) 
{
    // Keep annotation at 0,0,0, individual path targets are in world coordinates
    // Store the position for drawing of path 
    _posvel = posvel;

    apply();
}

void PlannedPath::onAnchorVisibilityChanged(bool visible)
{
    this->SetVisibility(visible);
}

void PlannedPath::onAnchorPlannedPathChanged(std::vector<PositionVelocity>& path)
{
    _path.clear();
    for (size_t i = 0; i < path.size(); ++i)
    {
        _path.push_back(path[i]);
    }

    apply();
}
