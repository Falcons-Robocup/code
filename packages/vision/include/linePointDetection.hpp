 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2016 Andre Pool and Geraldo Santiago
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef LINEPOINTDETECTION_HPP
#define LINEPOINTDETECTION_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "configurator.hpp"
#include "preprocessor.hpp"

#include <mutex>

typedef struct
{
	double angle;
	double radius;
} angleRadiusSt;

class linePointDetection
{

private:
	// pointers for access to other classes
	preprocessor *prep;
	configurator *conf;

	cv::Mat lineUnfilteredFrame, lineErodeFrame, lineDilateFrame, lineFrame, lineFrameExport;
	cv::Mat sunSpotFrame, sunSpotContoursFrame;
	cv::Mat lineBallMask;
	cv::Point centerPointPrevios;
	int cameraRadiusLineMaxPrevious;

	int linePixels;

	std::vector<positionStFl> inLinePixelsList; // contain the center line points, used by determine position // APOX remove
	std::vector<angleRadiusSt> linePointsRadianMeter, linePointsRadianMeterExport; // contains the list with line points in radians and meters
	std::vector<cv::Point> linePoints, linePointsExport; // contain the center line points, used by determine position
	std::vector<cv::Point> linePointsAll, linePointsAllExport; // used to show all found white pixels on grid lines as small grey lines in the round viewer
	std::vector<cv::Point> linePointsReject, linePointsRejectExport; // used to show rejected line points in viewer
	std::vector< std::vector<cv::Point> > spiralLines; // contains the spiral lines used to find the line pixels
	cv::Mat spiralLineFrame; // used visualize the spiral lines

	detPosSt goodEnoughLoc; // containing the last known position, used by robot1 to remove goal net from line pixels

	std::mutex exportMutex;

	void detectSunSpot( );
	void storePoint(cv::Point firstPoint, cv::Point lastPoint);
	void updateLinePointsRadianMeter();

public:
	linePointDetection(configurator *conf, preprocessor *prep);
	void update();
	void updateEdge();
	cv::vector<cv::Point> getLinePoints() { return linePointsExport; }
	cv::Mat getLinePointsFrame();
	cv::Mat getLinePointsAllFrame();
	cv::Mat getLinePointsRejectFrame();
	std::vector<angleRadiusSt> getLinePointsRadianMeter();
	cv::Mat getSpiralLineFrame();
	cv::Mat getSunSpotContours();
	cv::Mat getLineUnfiltered();
	cv::Mat getLineErode();
	cv::Mat getLineDilate();
	cv::Mat getSunSpot();
	cv::Mat getLine();
	int getLinePixels() { return linePixels; }
	void setGoodEnoughLoc( detPosSt value ) { goodEnoughLoc = value; }

};

#endif
