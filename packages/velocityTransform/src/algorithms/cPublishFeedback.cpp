// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPublishFeedback.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#include "int/algorithms/cPublishFeedback.hpp"

void cPublishFeedback::execute()
{
    TRACE_FUNCTION("");

    //vt_robot_data robotData;
    //_vtMain->_vtDataClass->getFeedbackRobotData(robotData);

    _vtMain->_vtDataClass->publishFeedback();

}


