// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef VISION_HPP
#define VISION_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

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

class visionLibrary {
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

	std::thread ballDetThread, cyanDetThread, magentaDetThread, ballPosThread, diagThread, locThread, obstDetThread,
			viewerThread;

public:
	visionLibrary(int robotIdArg, bool guiEnabled, int skipFrame);
	bool update();
};

#endif
