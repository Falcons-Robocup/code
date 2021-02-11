// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cMotionPlanningClient.hpp
 *
 * MotionPlanning client facility
 *
 *  Created on: Dec 01, 2018
 *      Author: Erik Kouters
 */

#ifndef CMOTIONPLANNINGCLIENT_HPP_
#define CMOTIONPLANNINGCLIENT_HPP_

// system includes
#include <vector>
#include <map>

// RTDB
#include "cRtDBClient.hpp"
#include "FalconsRtDB2.hpp"

#include "falconsCommon.hpp" // TODO fix type dealing abuse, use geometry package

#include "cWorldModelClient.hpp"


class cMotionPlanningClient : public cRtDBClient
{
  public:
    cMotionPlanningClient();
    ~cMotionPlanningClient();

    T_ACTION_RESULT executeAction(const T_ACTION actionData);
    double getTimeToBall(const uint8_t robotID);

  private:
    cWorldModelClient _wmClient;

};

#endif

