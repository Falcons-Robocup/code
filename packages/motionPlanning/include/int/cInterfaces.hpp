// Copyright 2017-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cInterfaces.hpp
 *
 *  Created on: Nov 19, 2017
 *      Author: Jan Feitsma
 */

#ifndef CINTERFACES_HPP_
#define CINTERFACES_HPP_


#include "int/cShootPlanningInterface.hpp"
#include "int/cPathPlanningInterface.hpp"
#include "int/cBallHandlingInterface.hpp"
#include "PathPlanningClient.hpp"
#include "MP_WorldModelInterface.hpp"

struct cInterfaces
{
    MP_WorldModelInterface *wm;
    PathPlanningClient *pp;
    MP_RTDBOutputAdapter *rtdbOutput;
};

#endif /* CINTERFACES_HPP_ */

