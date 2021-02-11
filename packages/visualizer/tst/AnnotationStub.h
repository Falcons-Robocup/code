// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AnnotationStub.h
 *
 *  Created on: December 7th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef ANNOTATIONSTUB_H
#define ANNOTATIONSTUB_H

#include <vtkAssembly.h>
#include <sstream>

// Internal:
#include "include/int/widgets/Field/Visualization/RobotVisualization.h"
#include "include/int/widgets/Field/Annotation/Annotation.h"

/*
* Stub that records the signals it got
*/
class AnnotationStub : public Annotation
{
    Q_OBJECT

public:
    void initialize(vtkSmartPointer<Visualization> anchor);

    bool onAnchorPositionChangedCalled = false;
    PositionVelocity anchorPosition;
    bool onAnchorVisibilityChangedCalled = false;
    bool anchorVisibility;
public Q_SLOTS:
    virtual void onAnchorPositionChanged(PositionVelocity& posvel) override;
    virtual void onAnchorVisibilityChanged(bool visible) override;
};

#endif // ANNOTATIONSTUB_H
