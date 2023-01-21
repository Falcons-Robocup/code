// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBDiagnosticsAdapter.hpp
 *
 *  Created on: Jun 26, 2022
 *      Author: Edwin Schreueder
 */

#ifndef RTDBDIAGNOSTICSADAPTER_HPP_
#define RTDBDIAGNOSTICSADAPTER_HPP_

#include <vector>

#include "FalconsRTDB.hpp"

class tpRTDBDiagnosticsAdapter
{
public:
    static tpRTDBDiagnosticsAdapter& getInstance()
    {
        static tpRTDBDiagnosticsAdapter instance;
        return instance;
    }

    void setDiagnosticsData();

private:
    tpRTDBDiagnosticsAdapter();

    RtDB2 *_rtdb;
};

#endif
