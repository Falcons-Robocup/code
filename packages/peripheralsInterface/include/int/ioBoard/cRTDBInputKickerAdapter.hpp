// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputKickerAdapter.hpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#ifndef INCLUDE_INT_ADAPTERS_CRTDBINPUTKICKERADAPTER_HPP_
#define INCLUDE_INT_ADAPTERS_CRTDBINPUTKICKERADAPTER_HPP_

#include "int/ioBoard/Kicker.hpp"
#include "FalconsRTDB.hpp"

class cRTDBInputKickerAdapter 
{
public:
    cRTDBInputKickerAdapter(Kicker& kicker);
    ~cRTDBInputKickerAdapter();

    void waitForKickerSetpoint();
    void getKickerSetpoint();

private:
    Kicker &kicker;
    int _myRobotId;
    RtDB2 *_rtdb;

};

#endif

