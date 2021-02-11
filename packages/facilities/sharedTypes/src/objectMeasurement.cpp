// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectMeasurement.cpp
 *
 *  Created on: Jul 9, 2018
 *      Author: Jan Feitsma
 */

#include "objectMeasurement.hpp"

bool objectMeasurement::operator==(const objectMeasurement &o) const
{
    return identifier == o.identifier;
}

bool objectMeasurement::operator<(const objectMeasurement &o) const
{
    if (identifier == o.identifier)
    {
        return false;
    }
    // allows sorting by decreasing confidence, and if equal, by increasing radius
    if (confidence == o.confidence)
    {
        return radius < o.radius;
    }
    return confidence > o.confidence;
}

