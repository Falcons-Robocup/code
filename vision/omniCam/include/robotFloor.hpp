// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBOTFLOOR_HPP
#define ROBOTFLOOR_HPP

#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "configurator.hpp"
#include "linePointDetection.hpp"

typedef struct {
	int xLeft;
	int xMiddle;
	int xRight;
	int yTop;
	int yMiddle;
	int yBottom;
} fieldSizeStruct;

typedef struct {
	int left;
	int right;
	int mark;
} penaltyXSizeStruct;

typedef struct {
	penaltyXSizeStruct xLeftArea;
	penaltyXSizeStruct xRightArea;
	int yTop;
	int yBottom;
} penaltySizeStruct;

typedef struct {
	int left;
	int right;
} goalXSizeStruct;

typedef struct {
	goalXSizeStruct xLeftArea;
	goalXSizeStruct xRightArea;
	int yTop;
	int yBottom;
	int xDepth; // APOX todo: remove
	int yPoleToCenter; // APOX todo: remove
} goalSizeStruct;

typedef struct {
	int arcRadius;
	int arcLineThickness;
} cornerSizeStruct;

class robotFloor {

private:
	// pointers for access to other classes
	configurator *conf;
	linePointDetection *linePoint;

	cv::Mat frameCost, inputFloor, outputFloor;
	cv::Mat refField;
	bool calibrateMode;
	int blurPixels;

	int metersToPixels;
	double remoteRatio; // used to convert robot pixels to remote viewer pixels

	floorSizeSt floorSize;
	fieldSizeStruct fieldSize;
	penaltySizeStruct penaltySize;
	goalSizeStruct goalSize;
	cornerSizeStruct cornerSize;
	int centerArcRadius;

	void createCalibrated();
	void createNormal();
	void blurFloor();
	void blurPixel(float blurValue);
	std::ofstream floorSizeFile;

	std::mutex exportMutex;

public:
	robotFloor(configurator *conf);
	uchar getCost(int x, int y) {
		return frameCost.at<uchar>(y, x);
	}
	void update();
	int getWidth();
	int getHeight();
	int getXFloorRight();
	int getYFloorBottom();
	cv::Mat getRefField();
	cv::Mat getCostField();
	bool inField(cv::Point position); // check if position is inside field lines
	double getRemoteRatio() {
		return remoteRatio;
	}
	int getMetersToPixels() {
		return metersToPixels;
	}
};

#endif
