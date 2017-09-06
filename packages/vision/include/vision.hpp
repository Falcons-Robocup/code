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

#ifndef VISION_HPP
#define VISION_HPP

#include "configurator.hpp"
#include "ballDetection.hpp"
#include "ballPossession.hpp"
#include "cFrameDiagnostics.hpp"
#include "obstacleDetection.hpp"
#include "determinePosition.hpp"
#include "preprocessor.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "multicastSend.hpp"
#include "robotFloor.hpp"
#include "viewer.hpp"
#include "observer.hpp"
#include <iostream>
#include <thread>

class visionLibrary
{
private:
	configurator *conf;
	ballDetection *cyanDet;
	ballDetection *magentaDet;
	ballDetection *ballDet;
	ballPossession *ballPos; // determine if ball is in ball handlers
	cFrameDiagnostics *diag;
	determinePosition *detPos;
	obstacleDetection *obstDet;
	linePointDetection *linePoint;
	localization *loc;
	multicastSend *multSend;
	preprocessor *prep;
	robotFloor *rFloor;
	viewer *view;

	cv::VideoCapture capture;
	int frameCurrent;
	bool guiEnabled;
	cv::Mat frameRaw;
	int robotId;
	bool useCamera;

	double previousSendTime;

	std::thread ballDetThread, cyanDetThread, magentaDetThread, ballPosThread, diagThread, locThread, obstDetThread, viewerThread;

public:
	visionLibrary( int robotIdArg, bool guiEnabled, int skipFrame );
	bool update( );
	void attach( observer *obs );
};

#endif
