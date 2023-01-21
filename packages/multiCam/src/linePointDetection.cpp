// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2022 Andre Pool and Geraldo Santiago
// SPDX-License-Identifier: Apache-2.0

// The goal of the Line Point Detection is to provide linepoints to fieldLut / Solver
// which are used to determine the position of the robot on the soccer field.

// The line points itself are created by the camera boards, but need to be converted,
// combined and filtered before providing them to the fieldLut / Solver.

// The following steps are performed by the linePointDetection:
//  - receive the linepoints from the camera boards (typical. 400 in total)
//    - each camera provides the line point as x and y in pixels related to that camera
//    - typical x value would 10 to 350 and y value would be 10 to 400
//  - de-warp the linepoints (camera pixels to meters)
//    - the de-warp function uses a lookup table that has been calibrated in advance
//  - map the de-warped linepoints of all cameras in the linePointField
//    - the linePointField typically would have a size of 800x800 pixels and represents
//      an area of 5x5 meters on the soccer field
//  - select a number (e.g. 100) of pixels in a homogeneous way from the linePointField
//  - show the selected pixels as blue, and the not selected pixels in as red in the linePointViewer
//  - use the selected line points to create a list in angle and radians
//  - provide the angle-radian list for the fieldLut / Solver

// Note:
// The linePointField is a representation of the real soccer field of e.g. 18 x 12 meters.
// The pixels of the linePointField represent the resolution of the field e.g. one pixel is 2 cm.
// That means a field of 18 x 12 meters is represented by 900 x 600 pixels in the linePointField.
// As well the field dimension (A and B) as the field resolution (metersToPixels) can be configured
// in the yaml file.

#include "linePointDetection.hpp"
#include <math.h> // atan2
#include <pwd.h>
#include <stdio.h>

#ifndef NOROS
#include "tracing.hpp"
#endif

// amount of steps in the 360 degrees circle
#define NRANGLES 100

using namespace cv;
using namespace std;

linePointDetection::linePointDetection(cameraReceive *camRecv, configurator *conf, Dewarper *dewarp[4],
		preprocessor *prep) {
	this->camRecv = camRecv;
	this->conf = conf;
	this->dewarp[0] = dewarp[0];
	this->dewarp[1] = dewarp[1];
	this->dewarp[2] = dewarp[2];
	this->dewarp[3] = dewarp[3];
	this->prep = prep;

	centerPointPrevios = Point(-1, -1); // force generating the mask the first time the findPointsInFrame is performed
	cameraRadiusLineMaxPrevious = -1;

	goodEnoughLoc.lastActive = INT_MAX;
	goodEnoughLoc.goodEnough = false;
	goodEnoughLoc.score = 1.0;

	linePointListExportMutex.unlock();
	exportMutex.lock();
	linePointsPolarExport.clear();
	exportMutex.unlock();
}

vector<angleRadiusSt> linePointDetection::getLinePointsPolar() {
	exportMutex.lock();
	vector<angleRadiusSt> retVal = linePointsPolarExport;
	exportMutex.unlock();
	return retVal;
}

// reject line pixels that might come from the goal keeper frame
bool linePointDetection::rejectGoalKeeperFrame(size_t cam, double polarAngle) {
	bool valid = true;

	// line points can only be valid when from before the keeper frame
	// reject everything that is on the keeper frame and behind
	// only need to for the left (cam1) and right camera (cam3)
	if( conf->getRobot() == 1 ) {
		// so it is the goal keeper
		// printf("angle %4.1f\n", 180.0 * polar.angle / M_PI);
		float angle_max = M_PI * 5.0 / 180.0;
		// left part (cam1) of the goal keeper frame
		if( (cam == 1) && (polarAngle > angle_max) ) {
			valid = false; // reject line pixels from keeper frame (or behind the keeper frame)
		}
		// right part (cam3) of the goal keeper frame
		if( (cam == 3) && (polarAngle < -angle_max) ) {
			valid = false; // reject line pixels from keeper frame (or behind the keeper frame)
		}
	}
	return valid;
}

// convert line points parallel on short axis from Cartesian to Polar
angleRadiusSt linePointDetection::cartesianToPolarLongAxis(size_t cam, linePointSt cartesian) {
	angleRadiusSt polar;

	// ignore close by line points (e.g. from reflections of the robot)
	if( cartesian.xBegin < conf->getLine().distanceOffset ) { // in pixels
		cartesian.xBegin = conf->getLine().distanceOffset;
	}

	// only use line points on the floor
	if( cartesian.xEnd > conf->getLine().distance ) { // in pixels
		cartesian.xEnd = conf->getLine().distance;
	}

	int16_t xField = 0;
	int16_t yField = 0;
	size_t xCamera = 0;
	size_t yCamera = 0;

	// TODO: make ranges configurable, see dewarpConfig.txt
	if( (cartesian.xEnd >= cartesian.xBegin) && (cartesian.yBegin < ROI_HEIGHT) && (cartesian.yEnd < ROI_HEIGHT) ) {
		polar.valid = true;

		if( cartesian.yBegin != cartesian.yEnd ) {
			printf("ERROR    : cam %zu scan long axis of camera, expected y begin of %d be the same as y end of %d\n",
					cam, cartesian.yBegin, cartesian.yEnd);
			polar.valid = false; // reject
		}

		// line found on one camera line, this is the normal case where we have an xBegin and xEnd which
		// use xBegin and y to calculate the angle and use the xBegin and xEnd to calculate the width
		int16_t xFieldBegin = 0;
		int16_t xFieldEnd = 0;
		int16_t yFieldTmp = 0;

		// convert from camera dimensions (pixels) to field dimensions (meters)
		bool ok = dewarp[cam]->transformFloor(cartesian.xBegin, cartesian.yBegin, yField, xFieldBegin); // closest by pixel, use this yField to calculate angle
		if( ok ) {
			ok = dewarp[cam]->transformFloor(cartesian.xEnd, cartesian.yBegin, yFieldTmp, xFieldEnd); // do not use yField, use this one to calculate the width
		}
		if( !ok ) {
			polar.valid = false; // outside calibration range, reject
		}

		// TODO: add checker for bogus values
		// TODO: figure out why width needs to be very large for far away points

		int16_t width = xFieldEnd - xFieldBegin;
		if( (width > conf->getLine().widthMin) && (width < conf->getLine().widthMax) ) {
			xField = xFieldBegin + (width / 2);
		} else {
			polar.valid = false; // reject
		}
		xCamera = (cartesian.xBegin + cartesian.xEnd) / 2;
		yCamera = cartesian.yBegin;

		// TODO: checkout why angle needs to be set negative over here
		// would expect the x and y would have the correct polarity and atan2 should then also be correct
		polar.angle = -atan2(yField, xField);

		if( polar.valid ) {
			polar.valid = rejectGoalKeeperFrame(cam, polar.angle);
		}

		// polar.angle = -polar.angle - CV_PI/2.0; // new calibration uses inverted angles - 90 degrees

		// cam 1 is left of cam 0 = counterclockwise
		polar.angle += cam * 2.0 * CV_PI / 4.0; // every camera is 90 degrees rotated
		polar.radius = sqrt(xField * xField + yField * yField);
		polar.xCamera = xCamera;
		polar.yCamera = yCamera;
	} else {
		// TODO: investigate if it is normal that this case does occur in real world testing
		polar.valid = false; // outside region, reject
	}

	return polar;
}

// convert line points parallel on short axis from Cartesian to Polar
angleRadiusSt linePointDetection::cartesianToPolarShortAxis(size_t cam, linePointSt cartesian) {
	angleRadiusSt polar;

	// ignore close by line points (e.g. from reflections of the robot)
	if( cartesian.xBegin < conf->getLine().distanceOffset ) { // in pixels
		cartesian.xBegin = conf->getLine().distanceOffset;
	}

	// only use line points on the floor
	if( cartesian.xEnd > conf->getLine().distance ) { // in pixels
		cartesian.xEnd = conf->getLine().distance;
	}

	int16_t xField = 0;
	int16_t yField = 0;
	size_t xCamera = 0;
	size_t yCamera = 0;

	// TODO: make ranges configurable, see dewarpConfig.txt
	if( (cartesian.xEnd >= cartesian.xBegin) && (cartesian.yBegin < ROI_HEIGHT) && (cartesian.yEnd < ROI_HEIGHT) ) {
		polar.valid = true;

		if( cartesian.xBegin != cartesian.xEnd ) {
			printf("ERROR    : cam %zu scan short axis of camera, expected x begin of %d be the same as x end of %d\n",
					cam, cartesian.xBegin, cartesian.xEnd);
			polar.valid = false; // reject
		}

		// line found from top to bottom of camera, this is the exception case where the line point is found right-angled on
		// scanning lines
		// use x and yBegin to calculate angle and use yBegin and yEnd to calculate the width
		// do it for the camera top to bottom
		int16_t xFieldTmp = 0;
		int16_t yFieldBegin = 0;
		int16_t yFieldEnd = 0;

		// convert from camera dimensions (pixels) to field dimensions (millimeters)
		bool ok = dewarp[cam]->transformFloor(cartesian.xBegin, cartesian.yBegin, yFieldBegin, xField);
		if( ok ) {
			ok = dewarp[cam]->transformFloor(cartesian.xBegin, cartesian.yEnd, yFieldEnd, xFieldTmp);
		}
		if( !ok ) {
			polar.valid = false; // outside calibration range, reject
		}

		yField = round((yFieldBegin + yFieldEnd) / 2.0);

		int16_t width = yFieldEnd - yFieldBegin + 1; // value in granularity of 20mm
		if( (width > conf->getLine().widthMin) && (width < conf->getLine().widthMax) ) {
			yField = yFieldBegin + (width / 2);
		} else {
			polar.valid = false; // reject
			// TODO: investigate why this case does not occur while testing
		}
		xCamera = cartesian.xBegin;
		yCamera = (cartesian.yBegin + cartesian.yEnd) / 2;

		// TODO: checkout why angle needs to be set negative over here
		// would expect the x and y would have the correct polarity and atan2 should then also be correct
		polar.angle = -atan2(yField, xField);

		if( polar.valid ) {
			polar.valid = rejectGoalKeeperFrame(cam, polar.angle);
		}

		// polar.angle = -polar.angle - CV_PI/2.0; // new calibration uses inverted angles - 90 degrees

		// cam 1 is left of cam 0 = counterclockwise
		polar.angle += cam * 2.0 * CV_PI / 4.0; // every camera is 90 degrees rotated
		polar.radius = sqrt(xField * xField + yField * yField);
		polar.xCamera = xCamera;
		polar.yCamera = yCamera;
	} else {
		// TODO: investigate if it is normal that this case does occur in real world testing
		polar.valid = false; // outside region, reject
	}
	return polar;
}

// collect the line points from the camera receiver and convert from Cartesian to Polar
void linePointDetection::update() {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	linePointsPolar.clear();
	vector<linePointSt> cartesian;

	// the line point detection thread is started when the long axis line point data
	// of cam0 is received
	// for the localization we need the line points from all 4 camera's (as well long axis as short axis)
	// because of the time synchronization difference of the camera's and network jitter
	// at this moment not all line points will be available (likely around 50%)
	// the period time is 25ms, when the cameras time synchronization application is running
	// in most cases all line points (>80%) are available within 5ms after receiving the long axis line
	// points from cam0
	// TODO: start line point detection from averaged grabber sync from cam0 instead of
	// the receiving time of the long axis line points (which has quite some jitter)
#ifndef NOROS
    TRACE("Starting sleep...");
#endif
	usleep(5000);
#ifndef NOROS
    TRACE("Done sleeping");
#endif
	linePointListExportMutex.lock(); // lock the list used by the camera viewer
	// convert the long axis line points from the 4 camera's
	vector<angleRadiusSt> linePointsPolarLongAxis[4];
	for( size_t cam = 0; cam < 4; cam++ ) {
		vector<linePointSt> cartesian = camRecv->getLinePointsLongAxis(cam);
		linePointList[cam] = cartesian; // list used by the camera viewer
		for( size_t ii = 0; ii < cartesian.size(); ii++ ) {
			angleRadiusSt polar = cartesianToPolarLongAxis(cam, cartesian[ii]);
			if( polar.valid ) {
				linePointsPolarLongAxis[cam].push_back(polar);
			}
		}
	}

	// convert the short axis line points from the 4 camera's
	vector<angleRadiusSt> linePointsPolarShortAxis[4];
	for( size_t cam = 0; cam < 4; cam++ ) {
		vector<linePointSt> cartesian = camRecv->getLinePointsShortAxis(cam);
		linePointList[cam].insert(linePointList[cam].end(), cartesian.begin(), cartesian.end()); // add to list used by the camera viewer
		for( size_t ii = 0; ii < cartesian.size(); ii++ ) {
			angleRadiusSt polar = cartesianToPolarShortAxis(cam, cartesian[ii]);
			if( polar.valid ) {
				linePointsPolarShortAxis[cam].push_back(polar);
			}
		}
	}
	linePointListExportMutex.unlock();

// To reduce CPU load there a limited amount of linePoints is used (defined yaml: linePointsNumberMaximal = default 100)
// The goal is to get a homogeneous distribution of the field (all camera's) with a preference for far away line pixels
// When there are more line points available then processed the following choice will be made:
// - for each camera 5 far away line points from long axis and 5 far away line points from short axis (total 4*2*5=40)
// - then for each camera 5 random from the long axis and 5 random from the short axis (total 4*2*5=40)
// - for the remaining a random choice will be made from the remaining pixels (default minimal 100)

	linePointsPolar.clear();
	vector<angleRadiusSt> linePointsPolarRemainder;
	size_t longAxis = 0;
	size_t shortAxis = 0;

	for( size_t cam = 0; cam < 4; cam++ ) {
		longAxis += linePointsPolarLongAxis[cam].size(); // count the total amount of line pixels from the long axis

		// sort the line pixel list, so the far away pixel is at the begin of the sorted list
		sort(linePointsPolarLongAxis[cam].begin(), linePointsPolarLongAxis[cam].end());
		size_t amount = linePointsPolarLongAxis[cam].size();

		// always use a number of the most distant line points
		size_t farAmount = amount;
		if( farAmount > 5 ) {
			farAmount = 5;
		}
		linePointsPolar.insert(linePointsPolar.end(), linePointsPolarLongAxis[cam].begin(),
				linePointsPolarLongAxis[cam].begin() + farAmount);

		// shuffle the remainder of the line points (vector start at begin + farAmount)
		random_shuffle(linePointsPolarLongAxis[cam].begin() + farAmount, linePointsPolarLongAxis[cam].end());

		// take an amount from the random remaining list (starting at begin + farAmount)
		ssize_t randomAmount = amount - farAmount;
		if( randomAmount > 5 ) {
			randomAmount = 5;
		}
		linePointsPolar.insert(linePointsPolar.end(), linePointsPolarLongAxis[cam].begin() + farAmount,
				linePointsPolarLongAxis[cam].begin() + farAmount + randomAmount);

		// put the left over points in the remainder list (starting at begin + farAmount + randomAmount
		linePointsPolarRemainder.insert(linePointsPolarRemainder.end(),
				linePointsPolarLongAxis[cam].begin() + farAmount + randomAmount, linePointsPolarLongAxis[cam].end());
	}

	for( size_t cam = 0; cam < 4; cam++ ) {
		shortAxis += linePointsPolarShortAxis[cam].size(); // count the total amount of line pixels from the short axis

		// sort the line pixel list, so the far away pixel is at the begin of the sorted list
		sort(linePointsPolarShortAxis[cam].begin(), linePointsPolarShortAxis[cam].end());
		// linePointsPolarAll.insert(linePointsPolarAll.end(), linePointsPolarShortAxis[cam].begin(), linePointsPolarShortAxis[cam].end());
		size_t amount = linePointsPolarShortAxis[cam].size();

		// always use a number of the most distant line points
		size_t farAmount = amount;
		if( farAmount > 5 ) {
			farAmount = 5;
		}
		linePointsPolar.insert(linePointsPolar.end(), linePointsPolarShortAxis[cam].begin(),
				linePointsPolarShortAxis[cam].begin() + farAmount);

		// shuffle the remainder of the line points
		random_shuffle(linePointsPolarShortAxis[cam].begin() + farAmount, linePointsPolarShortAxis[cam].end());

		// take an amount from the random remaining list (starting at begin + farAmount)
		ssize_t randomAmount = amount - farAmount;
		if( randomAmount > 5 ) {
			randomAmount = 5;
		}
		linePointsPolar.insert(linePointsPolar.end(), linePointsPolarShortAxis[cam].begin() + farAmount,
				linePointsPolarShortAxis[cam].begin() + farAmount + randomAmount);

		// put the left over points in the remainder list (starting at begin + farAmount + randomAmount
		linePointsPolarRemainder.insert(linePointsPolarRemainder.end(),
				linePointsPolarShortAxis[cam].begin() + farAmount + randomAmount, linePointsPolarShortAxis[cam].end());
	}

// shuffle all remaining points
	random_shuffle(linePointsPolarRemainder.begin(), linePointsPolarRemainder.end());

// add the random remaining ports at the end of the line point list
// Note: the localization algorithm will only use the first linePointsNumberMaximal (default 100) from this list
	linePointsPolar.insert(linePointsPolar.end(), linePointsPolarRemainder.begin(), linePointsPolarRemainder.end());

	exportMutex.lock();
	linePointsPolarExport = linePointsPolar;
	exportMutex.unlock();

}

vector<linePointSt> linePointDetection::getLinePoints(size_t camIndex) {
	if( camIndex >= 4 ) {
		printf("ERROR  : tried to get line points from camera index %zu, while the index goes to 3", camIndex);
		exit(EXIT_FAILURE);
	}
	linePointListExportMutex.lock();
	vector<linePointSt> retVal = linePointList[camIndex];
	linePointListExportMutex.unlock();
	return retVal;
}
