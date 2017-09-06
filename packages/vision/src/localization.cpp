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

#include "localization.hpp"

using namespace std;
using namespace cv;

localization::localization(configurator *conf, determinePosition *detPos, linePointDetection *linePoint, preprocessor *prep) {
	this->conf = conf;
	this->detPos = detPos;
	this->linePoint = linePoint;
	this->prep = prep;
	busy = false;

	start = getTickCount();
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii=0; ii<timeSize; ii++ ) { // initialize all localization time moments with a default value
		time[ii] = start;
	}
	count = 0;
}

void localization::update( ) {
	// determine if we need to use the special calibrate mode
	if( conf->getCalibrateMode() ) {
		linePoint->updateEdge();
	} else {
		linePoint->update();
	}

	detPos->pointsToPosition();

	// keep track of the last number of times when the localization was performed to be able to calculate the average localization FPS
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii=timeSize-1; ii>0 ; ii-- ) {
		time[ii] = time[ii-1];
	}
	time[0] = getTickCount();

	// calculate FPS
	fps = timeSize * 1.0 /((time[0] - time[timeSize-1])/getTickFrequency());

	// keep track how many frames have been processed
	count++;

	// inform higher level thread has finished
	busy = false;
}
