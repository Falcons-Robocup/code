// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AnnotationStub.cpp
 *
 *  Created on: December 7th, 2016
 *      Author: Diana Koenraadt
 */

#include "AnnotationStub.h"

void AnnotationStub::initialize(vtkSmartPointer<Visualization> anchor)
{
    Annotation::initialize(anchor);
}

void AnnotationStub::onAnchorPositionChanged(PositionVelocity& posvel)
{
    onAnchorPositionChangedCalled = true;
    anchorPosition = posvel;
}

void AnnotationStub::onAnchorVisibilityChanged(bool visible)
{
    onAnchorVisibilityChangedCalled = true;
    anchorVisibility = visible;
}
