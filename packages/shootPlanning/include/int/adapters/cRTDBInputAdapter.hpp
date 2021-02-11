// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.hpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#ifndef CRTDBINPUTADAPTER_HPP_
#define CRTDBINPUTADAPTER_HPP_

#include "int/cShootPlanner.hpp"

#include "FalconsRtDB2.hpp"

class cRTDBInputAdapter
{
  public:
    cRTDBInputAdapter() { };
    cRTDBInputAdapter(cShootPlanner *shootPlanner);
    ~cRTDBInputAdapter();

    void waitForShootSetpoint();
    void getShootSetpoint();

  private:
    int _myRobotId;
    RtDB2 *_rtdb;
    cShootPlanner* _shootPlanner;

};

#endif

