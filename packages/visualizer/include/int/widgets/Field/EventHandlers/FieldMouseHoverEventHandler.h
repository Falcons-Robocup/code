// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * FieldMouseHoverEventHandler.h
 *
 *  Created on: November 13, 2016
 *      Author: Diana Koenraadt
 */

#ifndef FIELDWIDGET_MOUSEHOVER_EVENTHANDLER_H
#define FIELDWIDGET_MOUSEHOVER_EVENTHANDLER_H

// Internal:
#include "int/widgets/Field/FieldWidget3D.h"

/*
* This eventhandler, if enabled, will make all actors transparent, *except* those 'seen' by the robot that is being hovered over.
* (If applicable. If no robot is below the mouse hover, nothing happens.)
*/
class FieldMouseHoverEventHandler : public FieldEventHandler
{
    Q_OBJECT
public:
    FieldMouseHoverEventHandler(FieldWidget3D* field);

protected:
    virtual void handle(QEvent* event) override;
};

#endif // FIELDWIDGET_MOUSEHOVER_EVENTHANDLER_H
