 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

// Application to interface frontVision with ros

#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdio.h>

#include "FalconsCommon.h"
#include "frontVision.hpp"
#include "adapters/cROSAdapter.hpp"
#include "adapters/cDiagAdapter.hpp"
#include "ext/cDiagnosticsEvents.hpp"

using namespace std;

bool shutDown = false; // global variable used to stop the main loop when cntr-c signal

// interrupter signal received e.g. ctrl-c
void sigIntHandler(int s) {
	printf("received interrupt signal, probably ctrl-c, do a clean shutdown\n");
	shutDown = true;
}

int main(int argc, char *argv[]) {

    /*
	if(getRobotNumber() != 1)
	{
		TRACE("sleeping forever since robot is not goal keeper");
		sleep(3600);
		// otherwise jobManager keeps complaining that 'process frontVision has died'
		return 0;
	}*/

	// redirect cntr-c signal to do a clean shutdown
	signal(SIGINT, sigIntHandler);

	frontVision fVision;

	size_t printCounter = 0;

	cDiagAdapter diagAdapter;
	cROSAdapter rosAdapter(&diagAdapter);

	ros::init(argc, argv, "frontVision");
	ros::NodeHandle n;

    //TRACE_ERROR("frontVision (re-)starting"); // must be after ros::init

	rosAdapter.initializeAdapter();

	while( ! shutDown && fVision.process() )
	{ // stop when cntr-c
    	diagAdapter.clear();
    	diagAdapter.setFPS(fVision.getStatus().fps);

    	std::vector<ballPositionType> adapterBalls;
    	std::vector<objectSt> visionBalls = fVision.getBalls();

		for (auto it = visionBalls.begin(); it != visionBalls.end(); it++)
		{
			ballPositionType ball;
			ball.setAngle(it->angle);
			ball.setRadiusAngle(it->radiusAngle);
			ball.setSize(it->size);

			adapterBalls.push_back(ball);
		}
		rosAdapter.setBall(adapterBalls);
    	//diagAdapter.send(); // JFEI disabled in wmV2

		// show every second the found balls
		if( printCounter > 29 ) {
			printf( "fps %3.1f | amount%2lu |", fVision.getStatus().fps, fVision.getBalls().size());
			for( size_t ii = 0; ii < fVision.getBalls().size(); ii++ ) {
				// print only the largest 5 balls
				if( ii < 5 ) {
					printf( " %6.2f", fVision.getBalls()[ii].angle );
					// the angle, in Radians, measured from the center of the screen to the ball
					// the Y axis represents the zero angle, balls left of Y axis result in a positive angle,
					// balls right of Y axis result in negative angle (counter clockwise up)
					// Note: for easier interpretation the viewer shows the angle in degrees!
					printf( " %5.2f", fVision.getBalls()[ii].radiusAngle );
					// the radiusAngle, in Radians, is measured from the center of the screen to the ball
					// Note: for easier interpretation the viewer and the explanation below is in degrees
			 		//  - a ball straight to the robot has a radiusAngle of 0 degrees
			 		//  - a ball on the left side or right of the x-axis has a radiusAngle of +60 degrees (+ for as well left as right)
			 		//  - a ball halfway on the x-axis has a radiusAngle of +30 degrees, TODO: improve model
			 		//  - the vertical angle is 120*720/1280 = 67.5 degrees
			 		//  - a ball on the top or bottom of the y-axis has a radiusAngle of +34 degrees(+ for as well top as bottom)
					printf( " %7.5f |", fVision.getBalls()[ii].size );
					// the size of the ball is scaled to the total view of the camera
					// so a size of 1.0 means the ball fills the complete view of the camera
					// Note: for easier interpretation the viewer shows the size of the ball in pixels
				}
			}
			printf("\n");
			printCounter = 0;
		} else {
			printCounter++;
		}
	}

	return 0;
}

