// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2022 Andre Pool and Geraldo Santiago
// SPDX-License-Identifier: Apache-2.0

#ifndef LINEPOINTDETECTION_HPP
#define LINEPOINTDETECTION_HPP

#include <mutex>

#include <opencv2/opencv.hpp>

#include "configurator.hpp"
#include "cameraReceive.hpp"
#include "dewarp.hpp"
#include "preprocessor.hpp"

struct angleRadiusSt {
	double angle;
	double radius;
	size_t xCamera;
	size_t yCamera;
	bool valid;
	bool operator<(const angleRadiusSt &val) const {
		// sorting this struct is performed on radius (distance)
		return radius > val.radius;
	}
};

class linePointDetection {

private:
	// pointers for access to other classes
	configurator *conf;
	cameraReceive *camRecv;
	Dewarper *dewarp[4];
	preprocessor *prep;

	cv::Mat lineBallMask;
	cv::Point centerPointPrevios;
	int cameraRadiusLineMaxPrevious;

	std::mutex linePointListExportMutex;
	std::vector<linePointSt> linePointList[4]; // used by viewer to show what localization is using
	std::vector<angleRadiusSt> linePointsPolar, linePointsPolarExport; // contains the list with line points in radians and millimeters

	detPosSt goodEnoughLoc; // containing the last known position, used by robot1 to remove goal net from line pixels

	std::mutex exportMutex;

	bool rejectGoalKeeperFrame(size_t cam, double polarAngle);
	angleRadiusSt cartesianToPolarLongAxis(size_t cam, linePointSt cartesian);
	angleRadiusSt cartesianToPolarShortAxis(size_t cam, linePointSt cartesian);

public:
	linePointDetection(cameraReceive *camRecv, configurator *conf, Dewarper *dewarp[4], preprocessor *prep);
	void update();
	std::vector<angleRadiusSt> getLinePointsPolar();
	void setGoodEnoughLoc(detPosSt value) {
		goodEnoughLoc = value;
	}
	std::vector<linePointSt> getLinePoints(size_t camIndex);

};

#endif
