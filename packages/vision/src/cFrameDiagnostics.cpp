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

// Collect data for diangostics

#include "cFrameDiagnostics.hpp"
#include <iomanip>

#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <pwd.h>

using namespace cv;
using namespace std;

cFrameDiagnostics::cFrameDiagnostics( configurator *conf, robotFloor *rFloor, viewer *view )
#ifndef NOROS
    : diagFrameSender(diagnostics::DIAG_VISION_RFRAME, 0)
#endif
{
	this->conf = conf;
	this->rFloor = rFloor;
	this->view = view;
    
	fileNameCounter = 0;

	// initialize to prevent sending bogus data through diagnostics channel
	roundFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	floorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);

	// collect the values from the diagnostics section in the yaml file
	struct passwd *pw = getpwuid( getuid( ) );
	string configFile( "" );
	configFile.append( pw->pw_dir );
	configFile.append( "/falcons/code/packages/vision/vision.yaml" );
	FileStorage fs( configFile, FileStorage::READ );

	FileNode diagConf = fs["diagnostics"];
	period = (double)diagConf["period"];
	int pngCompress = (int)diagConf["pngCompress"];
	int jpgCompress = (int)diagConf["jpgCompress"];

	pngCompresionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
	pngCompresionParams.push_back(pngCompress);

	jpgCompressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
	jpgCompressionParams.push_back(jpgCompress);

	nextTick = getTickCount( ) + (int64)(period * getTickFrequency( )); // wait one period before sending the first figure (at time 0 the picture is black)

	busy = false;
}

void cFrameDiagnostics::update() {
#ifdef NOROS
	updateAlone();
#else
	updateROS();
#endif
	busy = false;
}


// show both viewers and manage short keys
void cFrameDiagnostics::updateAlone() {
    // TODO: when writing files, use logdir, or at least something else which does not show up as git work-in-progress
	if( getTickCount( ) > nextTick ) {
		nextTick = getTickCount( ) + (int64)(period * getTickFrequency( ));

		// round camera image with overlay
		roundFrame = view->getRoundFrame( );
		sprintf(filename, "roundFrame%05d.jpg", fileNameCounter);
		imwrite(filename, roundFrame, jpgCompressionParams);

#ifdef NONO
		// jpg results into a significant smaller file then png
		sprintf(filename, "roundFrame%05d.png", fileNameCounter);
		imwrite(filename, roundFrame, pngCompresionParams);
#endif

		// rectangular floor image with expected robot position, obstacles and ball(s)
		// Note: in stead of sending the complete image, the floor can also be send as "vector" information and be regenerated on the far end
		floorFrame = view->getFloorFrame( );
		sprintf(filename, "floorFrame%05d.jpg", fileNameCounter);
		imwrite(filename, floorFrame, jpgCompressionParams);

		fileNameCounter++;
	}
}

#ifndef NOROS

void cFrameDiagnostics::updateROS() {
	if( getTickCount( ) > nextTick ) {
		nextTick = getTickCount( ) + (int64)(period * getTickFrequency( ));

		// round camera image with overlay
		roundFrame = view->getRoundFrame( );
		sprintf(filename, "/var/tmp/visionFrame.jpg");
		imwrite(filename, roundFrame, jpgCompressionParams);
		
		// read back the file as bytestream
		rosMsgs::t_diag_vision_rframe msg;
        std::ifstream input(filename, std::ios::binary);   
        std::copy( 
            std::istreambuf_iterator<char>(input), 
            std::istreambuf_iterator<char>( ),
            std::back_inserter(msg.data));
        
		// send over diagnostics
        // JFEI 20170115 disabled: huge packets easily get lost in wifi; we do not have visualization anyway
        // diagFrameSender.set(msg);
	}
}

#endif

