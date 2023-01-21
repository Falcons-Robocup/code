// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.hpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#ifndef CRTDBINPUTADAPTER_HPP_
#define CRTDBINPUTADAPTER_HPP_

#include "int/motors/PeripheralsInterfaceData.hpp"

#include "FalconsRTDB.hpp"

class cRTDBInputAdapter
{
  public:
    cRTDBInputAdapter(PeripheralsInterfaceData& piData);
    ~cRTDBInputAdapter();

    void getMotorVelocitySetpoint();
    void getBallHandlersMotorSetpoint();

  private:
    int _myRobotId;
    RtDB2 *_rtdb;
    PeripheralsInterfaceData &_piData;

};

#endif

