 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanningClient.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */


#include "ext/PathPlanningClient.hpp"
#include "int/PathPlanning.hpp"
#include "int/adapters/RTDBInputAdapter.hpp"
#include "int/adapters/RTDBOutputAdapter.hpp"

// common Falcons headers
#include "ConfigRTDBAdapter.hpp" // configuration
#include "tracing.hpp" // tracing
#include "falconsCommon.hpp" // common


// globals... better to put in this file, instead of making everything
// visible to motionPlanning via PathPlanningClient.hpp
RTDBInputAdapter *g_rtdbInputAdapter;
RTDBOutputAdapter *g_rtdbOutputAdapter;
ConfigRTDBAdapter<ConfigPathPlanning> *g_configAdapter;
PathPlanning *g_pp;


PathPlanningClient::PathPlanningClient()
{
    INIT_TRACE;

    // setup adapters
    TRACE("setting up RTDBInputAdapter");
    g_rtdbInputAdapter = new RTDBInputAdapter();
    TRACE("setting up RTDBOutputAdapter");
    g_rtdbOutputAdapter = new RTDBOutputAdapter(true);
    TRACE("setting up ConfigRTDBAdapter");
    g_configAdapter = new ConfigRTDBAdapter<ConfigPathPlanning>(CONFIG_PATHPLANNING);
    std::string configFile = determineConfig("PathPlanning");
    g_configAdapter->loadYAML(configFile);

    // setup PathPlanning, connect adapters
    TRACE("setting up PathPlanning");
    g_pp = new PathPlanning(g_configAdapter, g_rtdbInputAdapter, g_rtdbOutputAdapter);
}

PathPlanningClient::~PathPlanningClient()
{
    delete g_pp;
    delete g_configAdapter;
    delete g_rtdbOutputAdapter;
    delete g_rtdbInputAdapter;
}

actionResultTypeEnum PathPlanningClient::iterate()
{
    if (g_pp != NULL)
    {
        if (_overrideRzLimits)
        {
            g_pp->setRzLimitsOverride(_overrideRzMaxVel, _overrideRzMaxAcc);
        }
        actionResultTypeEnum result = g_pp->iterate();
        g_pp->unsetRzLimitsOverride();
        return result;
    }
    return actionResultTypeEnum::FAILED;
}

void PathPlanningClient::spin()
{
    if (g_rtdbInputAdapter == NULL) return;
    if (g_pp == NULL) return;
    // run
    TRACE("starting spin");
    bool shutdown = false;
    while (!shutdown)
    {
        TRACE("wait");
        g_rtdbInputAdapter->waitForMotionSetpoint();
        TRACE("iterate");
        (void)g_pp->iterate(); // ignore output ...
        // NOTE: this function was used when pathPlanning ran as its own process
        // but now motionPlanning calls pathPlanning as library via iterate()
        // so it can use its return feedback PASSED/RUNNING/FAILED
    }
    TRACE("done");
}

