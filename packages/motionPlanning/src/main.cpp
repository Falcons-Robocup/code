// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#include <boost/thread.hpp>

#include "../include/int/adapters/MP_RTDBInputAdapter.hpp"
#include "../include/int/adapters/MP_RTDBOutputAdapter.hpp"
#include "../include/int/MP_WorldModelInterface.hpp"
#include "int/cMotionPlanner.hpp"

#include "cDiagnostics.hpp"


int main(int argc, char **argv)
{
    INIT_TRACE("motionPlanning");

    try
    {

        // setup PathPlanning
        TRACE("setting up PathPlanning");
        PathPlanningClient pp;

        MP_WorldModelInterface *wmInterface = new MP_WorldModelInterface();
        MP_RTDBOutputAdapter *rtdbOutput = new MP_RTDBOutputAdapter();
        cMotionPlanner motionPlanner(wmInterface, &pp, rtdbOutput);
        
        // input adapter triggers motionPlanner to start executing
        MP_RTDBInputAdapter *rtdbInput = new MP_RTDBInputAdapter(&motionPlanner);

        
        rtdbInput->waitForActionData();
        
        // cleanup
        delete rtdbOutput;
        delete rtdbInput;
        delete wmInterface;
                
    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
        return 1;
    }
    return 0;
}


