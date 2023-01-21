// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Modify the raw input input image by the following steps
// flip (mirror)
// crop (remove unused areas), this can be controlled dynamic to center to the mirror
// convert to HSV
// filter for lines

#include "preprocessor.hpp"

using namespace std;
using namespace cv;

preprocessor::preprocessor(configurator *conf) {
	this->conf = conf;

	// initialize with the correct size, so the viewer can request them even if the update function was not performed
	// TODO if 720, 1280 does not need to be swapped
	full = Mat::zeros(720, 1280, CV_8UC3);
	bgr = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	exportMutex.lock();
	bgrExport = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	hsv = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	exportMutex.unlock();

	start = getTickCount();
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii = 0; ii < timeSize; ii++ ) { // initialize all frame capture time moments with a default value
		time[ii] = start;
	}
	count = 0;
	uptime = 0.0;
	fps = 0.0;
}

// flip, crop and convert BGR to HSV
void preprocessor::update(Mat inputImage) {
	full = inputImage;
	flip(full, full, 1); // compensate for the mirror by mirror around the y-axis

	// Prepare the region of interest
	// This will be different for each camera
	// 681 pixels, exact center pixel is 340, range 0 to 680
	int viewPixels = conf->getViewPixels();
	int xLeft = conf->getXOpticalBias() - 60 + (1280 - viewPixels) / 2; // bias value - bias center + unused left border
	if( xLeft < 0 ) {
		xLeft = 0;
	} // do not access data outside frameCamera matrix
	if( xLeft > (1280 - viewPixels) ) {
		xLeft = 1280 - viewPixels;
	}
	int yTop = conf->getYOpticalBias() - (720 - viewPixels + 1) / 2 + (720 - viewPixels + 1) / 2; // bias - bias center + unused top border
	if( yTop < 0 ) {
		yTop = 0;
	}
	if( yTop > (720 - viewPixels) ) {
		yTop = 720 - viewPixels;
	} // 719-681=038

	// get the region of interest by removing a part from the top, right, bottom and left
	// Note: bgr is using the same "memory" as full which uses the same as the input image (capture device)
	bgr = full(Rect(xLeft, yTop, viewPixels, viewPixels));

	exportMutex.lock();
	// used cloned version for export because BGR has relation with the input image (capture device)
	bgrExport = bgr.clone();
	// convert from BGR color space to HSV color space
	cvtColor(bgr, hsv, COLOR_BGR2HSV);
	exportMutex.unlock();

	// keep track of the last number of times when the localization was performed to be able to calculate the average localization FPS
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii = timeSize - 1; ii > 0; ii-- ) {
		time[ii] = time[ii - 1];
	}
	time[0] = getTickCount();

	// calculate FPS
	uptime = (time[0] - start) / getTickFrequency();
	fps = timeSize * 1.0 / ((time[0] - time[timeSize - 1]) / getTickFrequency());

	// keep track how many frames have been processed
	count++;
}

cv::Mat preprocessor::getBgr() {
	exportMutex.lock();
	cv::Mat retVal = bgrExport.clone();
	exportMutex.unlock();
	return retVal;
}
cv::Mat preprocessor::getHsv() {
	exportMutex.lock();
	cv::Mat retVal = hsv.clone();
	exportMutex.unlock();
	return retVal;
}
