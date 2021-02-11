// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include "BallDistanceEstimator.hpp"

#include <mutex>

#include "object.hpp" // commonality between balls and obstacles
typedef objectSt ballSt;


class ballDetection {

private:
	cameraReceive *camRecv;
	configurator *conf;
	Dewarper *dewarp;
	preprocessor *prep;
    BallDistanceEstimator *_distanceEstimator = NULL;

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

	std::vector<observer*> vecObservers; // only for Ros
	void notifyNewPos(); // only for Ros
	void notifyBallPossession(bool ballPossession); // only for Ros
	void findClosestLineGroups();

public:

	ballDetection(cameraReceive *camRecv, configurator *conf, Dewarper *dewarp, BallDistanceEstimator *distanceEstimator, preprocessor *prep, size_t type,
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
