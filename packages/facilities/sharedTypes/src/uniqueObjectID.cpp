// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectMeasurement.cpp
 *
 *  Created on: Jul 9, 2018
 *      Author: Jan Feitsma
 */

#include "uniqueObjectID.hpp"

bool uniqueObjectID::operator == (const uniqueObjectID &obj) const
{
    return (robotID  == obj.robotID) &&
    	   (uniqueID == obj.uniqueID);
}

bool uniqueObjectID::operator < (const uniqueObjectID &obj) const
{
    return (robotID < obj.robotID) ||
    	   (robotID == obj.robotID && uniqueID < obj.uniqueID);
}

