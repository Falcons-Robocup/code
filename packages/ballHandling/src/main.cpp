 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */


#include <boost/thread.hpp>

#include "int/ballHandlingControl.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "ConfigRTDBAdapter.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

boost::thread _workerThreadWaitForBallHandlersFeedback;


int main(int argc, char **argv)
{
    INIT_TRACE;

    try
    {
        // setup adapters
        TRACE("setup adapters");
        cRTDBInputAdapter rtdbInputAdapter;
        cRTDBOutputAdapter rtdbOutputAdapter;
        ConfigRTDBAdapter<ConfigBallHandling> configAdapter(CONFIG_BALLHANDLING);
        std::string configFile = determineConfig("BallHandling");
        configAdapter.loadYAML(configFile);
        TRACE("done setup adapters");
        WRITE_TRACE;

        // control algorithm
        TRACE("setup ballHandlingControl");
        ballHandlingControl controller(rtdbOutputAdapter, &configAdapter);
        TRACE("done setup ballHandlingControl");
        WRITE_TRACE;

        rtdbInputAdapter = cRTDBInputAdapter(&controller);

        // wait for ballhandlers feedback in a separate thread
        _workerThreadWaitForBallHandlersFeedback = boost::thread(boost::bind(&cRTDBInputAdapter::waitForBallHandlersFeedback, &rtdbInputAdapter));

        // spin
        TRACE("spin");
        rtdbInputAdapter.waitForBallHandlersSetpoint();
        
        // cleanup
    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
        return 1;
    }
    return 0;
}

