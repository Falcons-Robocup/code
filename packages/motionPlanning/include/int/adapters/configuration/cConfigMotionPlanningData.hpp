// Copyright 2018-2021 Shepherd Takawira (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cConfigMotionPlanningData.hpp
 *
 *  Created on: Apr 19, 2018
 */

#ifndef CCONFIGMOTIONPLANNING_HPP_
#define CCONFIGMOTIONPLANNING_HPP_

#include "FalconsRTDB.hpp"
#include "ConfigRTDBAdapter.hpp"


class cConfigMotionPlanningData
{
public:
    cConfigMotionPlanningData();
    ~cConfigMotionPlanningData();

    ConfigMotionPlanning getConfiguration();

private:
    ConfigRTDBAdapter<ConfigMotionPlanning>* _configAdapter;
};


#endif /* CCONFIGMOTIONPLANNINGDATA_HPP_ */
