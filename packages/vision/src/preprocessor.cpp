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

// Modify the raw input input image by the following steps
// flip (mirror)
// crop (remove unused areas), this can be controlled dynamic to center to the mirror
// convert to hsv
// filter for lines

#include "preprocessor.hpp"

using namespace std;
using namespace cv;

preprocessor::preprocessor(configurator *conf) {
	this->conf = conf;

	// initialize with the correct size, so the viewer can request them even if the update function was not performed
	// TODO if 720, 1280 does not need to be swapped
	full = Mat::zeros( 720, 1280, CV_8UC3 );
	bgr = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC3 );
	exportMutex.lock();
	bgrExport = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC3 );
	hsv = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC3 );
	exportMutex.unlock();

	start = getTickCount();
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii=0; ii<timeSize; ii++ ) { // initialize all frame capture time moments with a default value
		time[ii] = start;
	}
	count = 0;
}

// flip, crop and convert BGR to HSV
void preprocessor::update(Mat inputImage){
	full = inputImage;
	flip(full, full, 1); // compensate for the mirror by mirror around the y-axis

	// Prepare the region of interest
	// This will be different for each camera
	// 681 pixels, exact center pixel is 340, range 0 to 680
	int viewPixels = conf->getViewPixels();
	int xLeft = conf->getXOpticalBias() - 60 + (1280-viewPixels)/2; // bias value - bias center + unused left border
	if( xLeft < 0 ) { xLeft = 0; } // do not access data outside frameCamera matrix
	if( xLeft > (1280 - viewPixels) ) { xLeft = 1280 - viewPixels; }
	int yTop = conf->getYOpticalBias() - (720-viewPixels+1)/2 + (720-viewPixels+1)/2; // bias - bias center + unused top border
	if( yTop < 0 ) { yTop = 0; }
	if( yTop > (720 - viewPixels) ) { yTop = 720 - viewPixels; } // 719-681=038

	// get the region of interest by removing a part from the top, right, bottom and left
	// Note: bgr is using the same "memory" as full which uses the same as the input image (capture device)
	bgr = full(Rect( xLeft, yTop, viewPixels, viewPixels) );

	exportMutex.lock();
	// used cloned version for export because bgr has relation with the input image (capture device)
	bgrExport = bgr.clone();
	// convert from BGR color space to HSV color space
	cvtColor(bgr, hsv, COLOR_BGR2HSV);
	exportMutex.unlock();

	// keep track of the last number of times when the localization was performed to be able to calculate the average localization FPS
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii=timeSize-1; ii>0 ; ii-- ) {
		time[ii] = time[ii-1];
	}
	time[0] = getTickCount();

	// calculate FPS
	uptime = (time[0] - start )/getTickFrequency();
	fps = timeSize * 1.0 /((time[0] - time[timeSize-1])/getTickFrequency());

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
