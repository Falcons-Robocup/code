 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <vtkInteractorObserver.h>
#include <vtkCellPicker.h>

// Internal:
#include "int/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.h"
#include "int/ConfigurationManager.h"

FieldMouseHoverEventHandler::FieldMouseHoverEventHandler(FieldWidget3D* field)
  : FieldEventHandler(field)
{
}

void FieldMouseHoverEventHandler::handle(QEvent* event)
{
    return; // JFEI disable, suspected segfault below
    if (event->type() == QEvent::MouseMove)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);

        // Determine click coordinate
        vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
        picker->SetTolerance(0.01);
        picker->Pick(mouseEvent->x(), mouseEvent->y(), 0, _field->getRenderer()); 

        // Add team robots to the picklist.
        for (uint8_t i = 1; i <= _NR_OF_ROBOTS_PER_TEAM; ++i)
        {
            picker->AddPickList(_field->getTeamRobot(i));
        }
        picker->PickFromListOn();

        // Retrieve the actor that was hovered over and determine if this constitutes a Falcons robot.
        vtkAssembly* picked = picker->GetAssembly();
        if (picked)
        {
            RobotVisualization* robot = static_cast<RobotVisualization*>(picked);
            // Start by setting it invisible.
            robot->VisibilityOff();
        }
    }
}

