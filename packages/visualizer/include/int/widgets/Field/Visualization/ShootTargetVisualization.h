// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ShootTargetVisualization.h
 *
 *  Created on: Jul 2, 2017
 *      Author: Coen Tempelaars
 */

#ifndef SHOOTTARGETVISUALIZATION_H
#define SHOOTTARGETVISUALIZATION_H

#include <vtkActor.h>
#include <vtkSmartPointer.h>

// Internal:
#include "Visualization.h"

/*
* Class that groups the actor belonging to a shoot target
*/
class ShootTargetVisualization : public Visualization
{
public:
    static ShootTargetVisualization* New()
    {
        return new ShootTargetVisualization();
    }

    ShootTargetVisualization();
    virtual ~ShootTargetVisualization() {};

    void setPosition(PositionVelocity& posvel);
    void visibilityOff();

private:
    vtkSmartPointer<vtkActor> _actor;
};

#endif // SHOOTTARGETVISUALIZATION_H
