 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

// TKOV: i added the vision.hpp over here, but i am not sure if an additinal path is required
// include for the vision class
#include "vision.hpp"

/* ROS global includes */
#include <dynamic_reconfigure/server.h>
#include <boost/lexical_cast.hpp>

/* ROS node-specific includes */
// TKOV: is this still required?
#include "vision/visionConfig.h"

#include "FalconsCommon.h"
#include "ext/cDiagnosticsEvents.hpp"
#include "observerRos.hpp"

using namespace cv;
using std::cout;
using std::endl;

int main(int argc, char** argv)
{
	int opt = 0;
	bool guiEnabled = true;

	int robotIdArg = 0;
	while( (opt = getopt(argc, argv, "ci:") ) != -1 )
	{
		switch(opt)
		{
			case 'c':
				guiEnabled = false;
				printf("INFO      : disable the GUI\n");
				break;
			case 'i':
				robotIdArg = atoi(optarg);
				printf("INFO      : using command argument to set robot id to %d\n", robotIdArg);
				break;
		}
	}

	visionLibrary *vis = new visionLibrary( robotIdArg, guiEnabled, 0 );

	/* Take care of ROS initialization */
	ros::init(argc, argv, "vision");

	TRACE_ERROR("Vision (re-)starting"); // must be after ros::init

	// attach the observer
	bool cameraCorrectlyMounted = false; // but for balls and obstacles it needs to be true?!?! TODO Andre please help, workaround for now in observerRos.cpp
    observerRos *observerObj = new observerRos(robotIdArg, cameraCorrectlyMounted, 5.0);
	vis->attach(observerObj);

	// TKOV: can you please check the section below?
	// i am not sure if these things are still required, or
	// otherwise maybe i deleted to much.
	// From vision point of view only vis->update() is required.
	// If needed check visionAlone.cpp as reference

	ros::NodeHandle n;
	dynamic_reconfigure::Server<vision::visionConfig> srv;
	dynamic_reconfigure::Server<vision::visionConfig>::CallbackType f;

	ros::Rate loop_rate(30);

	while( ros::ok() && vis->update() ) {
		// frame diagnostics (low frequent) -- disabled until further notice, need UDP layer redesign first, 40KB packets get broken
		//TRACE("frame diagnostics");
		//diagObj->updateROS();


		TRACE("spinning");
		ros::spinOnce();
		
		// diagnostics
		observerObj->sendDiagnostics();
		
		TRACE("sleeping");
		loop_rate.sleep();
	}

	return 0;
}
