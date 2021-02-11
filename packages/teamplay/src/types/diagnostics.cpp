// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagnostics.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#include "int/types/diagnostics.hpp"

using namespace teamplay;

diagnostics::diagnostics()
:  _aiming(false)
{
}

diagnostics::~diagnostics()
{
}

bool diagnostics::isAiming() const
{
    return _aiming;
}

Point2D diagnostics::getShootTarget() const
{
    return _shootTarget;
}


void diagnostics::setAiming(const bool aiming)
{
    _aiming = aiming;
}

void diagnostics::setShootTarget(const Point2D& target)
{
    _shootTarget = target;
}
