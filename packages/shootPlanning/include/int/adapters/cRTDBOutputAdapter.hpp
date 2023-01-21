// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBOUTPUTADAPTER_HPP_
#define CRTDBOUTPUTADAPTER_HPP_

#include "FalconsRTDB.hpp"

class cRTDBOutputAdapter
{
  public:
    cRTDBOutputAdapter();
    ~cRTDBOutputAdapter();

    void setKickerSetpoint(kickerSetpointTypeEnum kickerSetpointType, float kickerHeight, float kickerPower);

  private:
    int _myRobotId;
    FalconsRTDB *_rtdb = NULL;

};

#endif

