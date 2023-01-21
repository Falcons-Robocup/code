// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBOUTPUTADAPTER_HPP_
#define CRTDBOUTPUTADAPTER_HPP_

#include "int/motors/PeripheralsInterfaceData.hpp"

#include "FalconsRTDB.hpp"

class cRTDBOutputAdapter
{
  public:
    cRTDBOutputAdapter(PeripheralsInterfaceData& piData);
    ~cRTDBOutputAdapter();

    void setMotorFeedback();
    void setBallHandlersFeedback();
    void setInPlayFeedback();

  private:
    int _myRobotId;
    RtDB2 *_rtdb;
    PeripheralsInterfaceData &_piData;

};

#endif

