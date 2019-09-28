// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef GRAB_VIEW_HPP
#define GRAB_VIEW_HPP

#include <thread>

#include "cameraReceive.hpp"
#include "dewarp.hpp"

class grabView {
private:
	cameraReceive *rx;
    deWarper *D;

	// int markerTrackbar[MARKER_MAX]; // copy from the marker in the deWarp, for use with trackbar and imshow

	cv::Mat cameraFrame, deWarpFrame, lookupTable;
	std::vector<camLinePointT> linePointList;
	std::thread receiveThread;

	void getReceiveData();
	void drawCameraFrame();
	void drawLinePoints();
	void drawDeWarp();

public:
	grabView(deWarper *D);
	void update();
	void startReceiveThread();
};

#endif
