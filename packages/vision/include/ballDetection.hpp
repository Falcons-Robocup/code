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

#ifndef BALLDETECTION_HPP
#define BALLDETECTION_HPP

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/photo/photo_c.h"
#include "configurator.hpp"
#include "preprocessor.hpp"
#include "observer.hpp"

#include <mutex>

struct ballSt {
	int size;
	double radiusTan;
	double radiusLin;
	double angle;
	bool operator<( const ballSt& val ) const {
		// sorting this struct is performed on size (higher is better)
		return size > val.size;
	}
};

class ballDetection {

private:
	preprocessor *prep;
	configurator *conf;
	size_t type; // ball detection re-used for cyuan and meganta detection, type is used to distinguish

	cv::Mat contoursFrame;
	cv::Mat dilateFrame;
	cv::Mat erodeFrame;
	cv::Mat inRangeFrame; // used to search the balls
	cv::Mat viewerContoursFrame; // display the balls in the viewer

	std::vector<ballSt> positions;

	std::mutex exportMutex;

	bool busy;

	// List of observers for Obstacle position notifications
	std::vector<observer*> vecObservers; // only for Ros
	void notifyNewPos(); // only for Ros

public:

	ballDetection(configurator *conf, preprocessor *prep, size_t type );
	void update( bool viewer );

	std::vector<ballSt> getPositions();
	std::vector<ballSt> getPositionsExport();
	cv::Mat getContoursFrame();
	cv::Mat getDilateFrame();
	cv::Mat getErodeFrame();
	cv::Mat getInRangeFrame();
	cv::Mat getViewerContoursFrame();

	bool getBusy() { return busy; }
	void setBusy() { busy = true; }

	void attach(observer *observer);
	void detach(observer *observer);
};
#endif
