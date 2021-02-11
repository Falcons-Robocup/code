// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotLabel.h
 *
 *  Created on: November 20, 2016
 *      Author: Diana Koenraadt
 */

#ifndef ROBOTLABEL_H
#define ROBOTLABEL_H

#include <vtkFollower.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>
#include <vtkVectorText.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <sstream>

// Internal:
#include "int/widgets/Field/Visualization/RobotVisualization.h"
#include "Annotation.h"

/*
* Class that floats a label next to a robot
*/
class RobotLabel : public Annotation, public vtkFollower
{
public:
    static RobotLabel* New()
    {
        return new RobotLabel();
    }

    void initialize(int robotID, vtkRenderer *renderer, RobotVisualization* anchor);

public Q_SLOTS:
    virtual void onAnchorPositionChanged(PositionVelocity& posvel) override;
    virtual void onAnchorVisibilityChanged(bool visible) override;

private:
    void addLabel(int robotID, vtkRenderer* renderer);
};

#endif // ROBOTLABEL_H
