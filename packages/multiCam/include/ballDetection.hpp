 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef BALLDETECTION_HPP
#define BALLDETECTION_HPP

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"

#include "cameraReceive.hpp"
#include "configurator.hpp"
#include "dewarp.hpp"
#include "preprocessor.hpp"
#include "observer.hpp"

#include <mutex>

struct ballSt {
	int size; // amount of ball pixels
	int xClosestBy; // for visualization only
	int yCenter; // for visualization only
	cv::Rect rect; // for visualization only
	double radius; // width (related to pixels)
	double azimuth;
	double elevation;
	bool operator<(const ballSt& val) const {
		// sorting this struct is performed on size = amount of pixels (higher is better)
		return size > val.size;
	}
};

class ballDetection {

private:
	cameraReceive *camRecv;
	configurator *conf;
	deWarper *dewarp;
	preprocessor *prep;

	size_t camIndex;
	uint16_t width, height;
	size_t printCount;

	size_t type; // ball detection re-used for cyan and magenta detection, type is used to distinguish

	cv::Mat dilateFrame;
	cv::Mat erodeFrame;
	cv::Mat inRangeFrame; // used to search the balls
	cv::Mat possessionFrame;

	size_t possessionPixels;

	std::vector<ssize_t> closestPixels;
	std::vector<ssize_t> closestGroups;

	std::vector<ballSt> positions;
	std::vector<ballSt> positionsRemoteViewer;
	std::vector<linePointSt> ballPointList; // used by viewer to show what localization is using

	std::mutex positionsRemoteViewerExportMutex;
	std::mutex ballPointListExportMutex;
	std::mutex exportMutex;

	bool busy;

	// List of observers for Obstacle position notifications
	std::vector<observer*> vecObservers; // only for Ros
	void notifyNewPos(); // only for Ros
	void notifyBallPossession(bool ballPossession); // only for Ros
	void findClosestLineGroups();

public:

	ballDetection(cameraReceive *camRecv, configurator *conf, deWarper *dewarp, preprocessor *prep, size_t type,
			size_t cam);
	void keepGoing();
	void update();

	std::vector<linePointSt> getBallPoints();
	std::vector<ballSt> getPositions();
	std::vector<ballSt> getAndClearPositionsRemoteViewer();
	std::vector<ballSt> getPositionsExport();
	std::vector<ssize_t> getClosestPixels();
	std::vector<ssize_t> getClosestGroups();
	cv::Mat getPossessionFrame();
	cv::Mat getDilateFrame();
	cv::Mat getErodeFrame();
	cv::Mat getInRangeFrame();
	size_t getPossessionPixels() {
		return possessionPixels;
	}  // size of ball pixels when ball is near ball handler
	size_t getAmount();

	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}

	void attach(observer *observer);
	void detach(observer *observer);
};
#endif
