// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Annotation.cpp
 *
 *  Created on: November 22nd, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/Field/Annotation/Annotation.h"

void Annotation::initialize(Visualization* anchor) 
{
    connect(anchor, SIGNAL(signalPositionChanged(PositionVelocity&)), this, SLOT(onAnchorPositionChanged(PositionVelocity&)));
    connect(anchor, SIGNAL(signalPositionChanged(polygon2D&)), this, SLOT(onAnchorPositionChanged(polygon2D&)));
    connect(anchor, SIGNAL(signalVisibilityChanged(bool)), this, SLOT(onAnchorVisibilityChanged(bool)));
};
