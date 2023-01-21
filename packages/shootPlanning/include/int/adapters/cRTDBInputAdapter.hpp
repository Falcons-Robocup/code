// Copyright 2019-2021 Erik Kouters (Falcons)
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

#include "FalconsRTDB.hpp"

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
    FalconsRTDB *_rtdb = NULL;
    cShootPlanner* _shootPlanner = NULL;

};

#endif

