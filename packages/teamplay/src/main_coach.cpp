 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main_coach.cpp
 *
 *  Created on: Dec 06, 2016
 *      Author: Michel Koenen
 *
 *      fork from main.cpp, removed stuff which is not needed on Coach
 */

#define TEAMPLAY_NODENAME ("teamplay_coach")

/* System includes */
#include <stdexcept>
#include "ros/ros.h"

/* Worldmodel includes */
#include "rosMsgs/t_worldmodel_calculated.h"
#include "int/cRosAdapterWorldModel.hpp"

// Diagnostics and tracing
#include "cDiagnosticsEvents.hpp"
#include "tracer.hpp"

// Teamplay internal includes
#include "int/adapters/cControlInterfaceROS.hpp"
#include "int/utilities/trace.hpp"
#include "int/cTeamplayControlInterface.hpp"


namespace {
void trace_error (const std::string& file, const int line, const std::string& func, const std::string& msg)
    {
        falcons::eventType event(file, line, func);
        event.message = msg;
        diagnostics::diag_error(event);
    }

void trace_info (const std::string& file, const int line, const std::string& func, const std::string& msg)
    {
        falcons::eventType event(file, line, func);
        event.message = msg;
        diagnostics::diag_info(event);
    }

void trace (const std::string& file, const int line, const std::string& func, const std::string& msg)
    {
        falcons::eventType event(file, line, func);
        event.message = msg;
        trace_write_event(event);
    }
} // unnamed namespace


int main(int argc, char **argv)
{
    /* Install the Falcons specific trace handlers */
    teamplay::traceRedirect::getInstance().setTraceFunction(&trace);
    teamplay::traceRedirect::getInstance().setTraceErrorFunction(&trace_error);
    teamplay::traceRedirect::getInstance().setTraceInfoFunction(&trace_info);

	try
	{
		// ROS initialization
		ros::init(argc, argv, TEAMPLAY_NODENAME);
		ros::Time::init();
    	//TRACE_INFO("teamplay_coach starting");

		// setup the teamplay interface (e.g. for robot control)
		cControlInterfaceROS controlInterfaceROS;

		/* Loop forever */
		ros::spin();
	}
	catch (std::exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
	}
	catch (...)
    {
        TRACE_ERROR("Caught unknown exception!");
    }
}

void cb_worldModelUpdated(const worldModel::t_wmInfo::ConstPtr& msg)
{
    //Empty definition to satisfy compiler for the coach process.
}

