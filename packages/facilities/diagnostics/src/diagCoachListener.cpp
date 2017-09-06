 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * diagCoachListener.cpp
 *
 * Listen to all diagnostics ports on coach laptop and publish one topic for each one.
 * 
 *  Created on: Jan 05, 2016
 *      Author: Jan Feitsma
 */


#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "FalconsCommon.h"
#include "ext/cDiagnostics.hpp" 


void reconnectThread()
{
	diagnostics::cDiagnosticsReceiver recv;
	connectionType connection = GetPrimaryConnectionType();

	while(1)
	{
		if(connection != GetPrimaryConnectionType())
		{
			TRACE("Reconstructing diagnostic receiver connections");
			recv = diagnostics::cDiagnosticsReceiver();
			connection = GetPrimaryConnectionType();
		}
		boost::this_thread::sleep_for(boost::chrono::seconds{5});
	}
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "diagCoachListener");

    // construct receiver
    TRACE("constructing cDiagnosticsReceiver");
    
    /* Create receiving thread */
    boost::thread reconnectCheck(reconnectThread);

    // spin
    TRACE("spinning");
    ros::spin();
    TRACE("exit 0");
    return 0;
}

