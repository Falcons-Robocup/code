// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mainMatchMode.cpp
 *
 * Bridge worldModel to teamplay, only when in match mode.
 * If in test mode, the bridge is open, because robotCLI is expected to stimulate the applicable component.
 *
 *  Created on: Feb 12, 2019
 *      Author: Jan Feitsma
 */

#include "FalconsRtDB2.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"


int main(int argc, char **argv)
{
    // setup
    int myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    bool matchMode = true;
    auto rtdb = RtDB2Store::getInstance().getRtDB2(myRobotId, teamChar);
    rtdb->put(MATCH_MODE, &matchMode);

    // run
    while (true)
    {
        // wait for new worldModel (which we expect to be on fixed frequency)
        rtdb->waitForPut(ROBOT_STATE);
        
        // trigger teamplay?
        if (matchMode)
        {
            T_TP_HEARTBEAT beat = 0;
            rtdb->put(TP_HEARTBEAT, &beat);
        }
        
        // recalculate matchmode
        bool newMatchMode = false;
        int r = rtdb->get(MATCH_MODE, &newMatchMode);
        if (r == 0)
        {
            // behavior: normally, the MATCHMODE key does not exist and the boolean matchMode will always remain true
            // once robotCLI creates the key MATCHMODE and sets it to false, here matchMode becomes false
            if (matchMode != newMatchMode)
            {
                matchMode = newMatchMode;
                tprintf("switching to matchMode=%d", matchMode);
            }
        }
        
    }
    
    return 0;
}
