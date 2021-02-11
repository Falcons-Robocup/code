// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef MP_RTDBINPUTADAPTER_HPP_
#define MP_RTDBINPUTADAPTER_HPP_

#include <vector>

#include "FalconsRtDB2.hpp"
#include "cRtDBClient.hpp"

#include "int/cMotionPlanner.hpp"

class MP_RTDBInputAdapter
{
  public:
    MP_RTDBInputAdapter(cMotionPlanner *mp);
    ~MP_RTDBInputAdapter();

    // Data from Teamplay
    void getActionData();
    void waitForActionData();

  private:
    RtDB2 *_rtdb;
    int _myRobotId;

    cMotionPlanner* _motionPlanner;

};

#endif

