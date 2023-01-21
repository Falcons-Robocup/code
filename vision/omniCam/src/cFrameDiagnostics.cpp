// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Collect data for diagnostics

#include "cFrameDiagnostics.hpp"
#include <iomanip>

#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <pwd.h>

using namespace cv;
using namespace std;

cFrameDiagnostics::cFrameDiagnostics(configurator *conf, robotFloor *rFloor, viewer *view) {
	this->conf = conf;
	this->rFloor = rFloor;
	this->view = view;

	fileNameCounter = 0;

	// initialize to prevent sending bogus data through diagnostics channel
	roundFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	floorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);

	// collect the values from the diagnostics section in the yaml file
	struct passwd *pw = getpwuid(getuid());
	string configFile("");
	configFile.append(pw->pw_dir);
	configFile.append("/falcons/code/vision/omniCam/omniCam.yaml");
	FileStorage fs(configFile, FileStorage::READ);

	FileNode diagConf = fs["diagnostics"];
	period = (double)diagConf["period"];
	int pngCompress = (int)diagConf["pngCompress"];
	int jpgCompress = (int)diagConf["jpgCompress"];

	pngCompresionParams.push_back(cv::IMWRITE_PNG_COMPRESSION);
	pngCompresionParams.push_back(pngCompress);

	jpgCompressionParams.push_back(cv::IMWRITE_JPEG_QUALITY);
	jpgCompressionParams.push_back(jpgCompress);

	nextTick = getTickCount() + (int64)(period * getTickFrequency()); // wait one period before sending the first figure (at time 0 the picture is black)

	busy = false;
}

void cFrameDiagnostics::update() {
	updateAlone();
	busy = false;
}

// show both viewers and manage short keys
void cFrameDiagnostics::updateAlone() {
	// TODO: when writing files, use logdir, or at least something else which does not show up as git work-in-progress
	if( getTickCount() > nextTick ) {
		nextTick = getTickCount() + (int64)(period * getTickFrequency());

		// round camera image with overlay
		roundFrame = view->getRoundFrame();
		sprintf(filename, "roundFrame%05d.jpg", fileNameCounter);
		imwrite(filename, roundFrame, jpgCompressionParams);

#ifdef NONO
		// jpg results into a significant smaller file then png
		sprintf(filename, "roundFrame%05d.png", fileNameCounter);
		imwrite(filename, roundFrame, pngCompresionParams);
#endif

		// rectangular floor image with expected robot position, obstacles and ball(s)
		// Note: in stead of sending the complete image, the floor can also be send as "vector" information and be regenerated on the far end
		floorFrame = view->getFloorFrame();
		sprintf(filename, "floorFrame%05d.jpg", fileNameCounter);
		imwrite(filename, floorFrame, jpgCompressionParams);

		fileNameCounter++;
	}
}
