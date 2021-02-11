// Copyright 2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VisionLock.h
 *
 *  Created on: December 18th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef VISIONLOCK_H
#define VISIONLOCK_H

#include <vtkAssembly.h>
#include <vtkCamera.h>

// Internal:
#include "int/widgets/Field/Visualization/RobotVisualization.h"
#include "Annotation.h"

/*
* Class that shows an icon to indicate whether a vision lock was acquired or not
*/
class VisionLock : public Annotation, public vtkAssembly
{
    Q_OBJECT

public:
    static VisionLock* New()
    {
        return new VisionLock();
    }

    void initialize(RobotVisualization* anchor);

private:
    PositionVelocity _posvel;

public Q_SLOTS:
    virtual void onAnchorPositionChanged(PositionVelocity& posvel) override;
    virtual void onAnchorVisibilityChanged(bool visible) override;

private:
    void addIcon(double width, double height);
};

#endif // PLANNEDPATH_H
