// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Annotation.h
 *
 *  Created on: November 20th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef FIELD_ANNOTATION_H
#define FIELD_ANNOTATION_H

#include <QObject>
#include <vtkProp.h>

// Internal:
#include "int/types/PositionVelocity.h"
#include "int/widgets/Field/Visualization/Visualization.h"

// External:
#include "polygon2D.hpp"

/*
* Class for annotations in the 3d visualization
*/
class Annotation : public QObject
{
        Q_OBJECT
public:
    void initialize(Visualization* anchor);
    virtual ~Annotation() {}

public Q_SLOTS:
    virtual void onAnchorPositionChanged(PositionVelocity& posvel) {};
	virtual void onAnchorPositionChanged(polygon2D& posvel) {};
    virtual void onAnchorVisibilityChanged(bool visible) {};
};

#endif // FIELD_ANNOTATION_H
