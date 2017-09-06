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

#ifndef CFRAMEDIAGNOSTICS_HPP
#define CFRAMEDIAGNOSTICS_HPP

#include "configurator.hpp"
#include "robotFloor.hpp"
#include "viewer.hpp"
#ifndef NOROS
#include "rosMsgs/t_diag_vision_rframe.h"
#include "cDiagnostics.hpp"
#endif
#include <iostream>

class cFrameDiagnostics {
private:
	// pointers for access to other classes
	configurator *conf;
	robotFloor *rFloor;
	viewer *view;

	bool busy;
	cv::Mat roundFrame;
	cv::Mat floorFrame;

	int fileNameCounter;
	double period; // time between sending two images through diagnostics
	int64 nextTick; // time when next image is send through diagnostics

	char filename[200];
	std::vector<int> jpgCompressionParams;
	std::vector<int> pngCompresionParams;

#ifndef NOROS
    diagnostics::cDiagnosticsSender<rosMsgs::t_diag_vision_rframe> diagFrameSender;
#endif

public:
	cFrameDiagnostics(configurator *conf, robotFloor *rFloor, viewer *view);
	void update();
	void updateAlone();
#ifndef NOROS
	void updateROS();
#endif	
	bool getBusy() { return busy; }
	void setBusy() { busy = true; }

};

#endif
