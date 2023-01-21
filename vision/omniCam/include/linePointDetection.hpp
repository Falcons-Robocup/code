// Copyright 2014-2022 Andre Pool and Geraldo Santiago
// SPDX-License-Identifier: Apache-2.0

#ifndef LINEPOINTDETECTION_HPP
#define LINEPOINTDETECTION_HPP

#include <mutex>
#include <opencv2/opencv.hpp>

#include "configurator.hpp"
#include "preprocessor.hpp"

typedef struct {
	double angle;
	double radius;
} angleRadiusSt;

class linePointDetection {

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
	std::vector<std::vector<cv::Point> > spiralLines; // contains the spiral lines used to find the line pixels
	cv::Mat spiralLineFrame; // used visualize the spiral lines

	detPosSt goodEnoughLoc; // containing the last known position, used by robot1 to remove goal net from line pixels

	std::mutex exportMutex;

	void detectSunSpot();
	void storePoint(cv::Point firstPoint, cv::Point lastPoint);
	void updateLinePointsRadianMeter();

public:
	linePointDetection(configurator *conf, preprocessor *prep);
	void update();
	void updateEdge();
	std::vector<cv::Point> getLinePoints() {
		return linePointsExport;
	}
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
	int getLinePixels() {
		return linePixels;
	}
	void setGoodEnoughLoc(detPosSt value) {
		goodEnoughLoc = value;
	}

};

#endif
