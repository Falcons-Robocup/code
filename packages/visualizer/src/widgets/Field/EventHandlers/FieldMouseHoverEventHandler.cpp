// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

