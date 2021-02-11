// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPublishTarget.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#include "int/algorithms/cPublishTarget.hpp"

void cPublishTarget::execute()
{
    TRACE_FUNCTION("");

    _vtMain->_vtDataClass->publishTarget();

}


