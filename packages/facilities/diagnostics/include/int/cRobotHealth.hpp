// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRobotHealth.hpp
 *
 *  Created on: Dec 16, 2018
 *      Author: Jan Feitsma
 */

#ifndef CROBOTHEALTH_HPP_
#define CROBOTHEALTH_HPP_


#include "FalconsRtDB2.hpp"


class cRobotHealth
{
  public:
    cRobotHealth();
    ~cRobotHealth();
    
  private:
    RtDB2 *_rtdb = NULL;
    
    void runSlow();
    void runFast();
};

#endif

