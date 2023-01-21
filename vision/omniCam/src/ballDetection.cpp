// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "ballDetection.hpp"

using namespace std;
using namespace cv;

ballDetection::ballDetection(configurator *conf, preprocessor *prep, size_t type) {
	this->conf = conf;
	this->prep = prep;
	this->type = type; // ball detection re-used for cyan and magenta detection, type is used to distinguish

	exportMutex.lock();
	// initialize frames so even without running the update method the frames can be used by the viewer
	contoursFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	erodeFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	dilateFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	inRangeFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	viewerContoursFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);

	// make the result list empty so the first request does not get bogus data
	positions.clear();
	exportMutex.unlock();
	busy = false;
}

// Search for balls in the round viewer.
// Each ball position is available as angle, radius and size.
// The angle and radius are relative to the robot (and not to the field).
// An angle of 0 degrees means in front of the shooter.
// The angle is relative the the shooter (0 degrees) of the robot.
// The radius is distance from the center of the robot to the center of the ball.
// The radius is available in pixel distance and meter distance.
// The pixel distance (RadiusTan) is the distance in the round viewer and is only used for visual debugging.
// The meter distance (RadiusLIn) is the distance in the rectangular field viewer, this number used by WorldModel.
// Note: The radius is measured from the closest yellow pixel, but because of the camera position, this
// is roughly the same as the center of the ball.
// The size is the amount of yellow pixels (in round viewer) of that ball.
void ballDetection::update(bool viewer) {
	// apply the color filter to the HSV input image
	Scalar ballMin(conf->getBall(type).hue.min, conf->getBall(type).sat.min, conf->getBall(type).val.min);
	Scalar ballMax(conf->getBall(type).hue.max, conf->getBall(type).sat.max, conf->getBall(type).val.max);
	exportMutex.lock();
	inRange(prep->getHsv(), ballMin, ballMax, inRangeFrame); // filter ball out
	erode(inRangeFrame, erodeFrame, conf->getBall(type).erode); // reduce
	dilate(erodeFrame, dilateFrame, conf->getBall(type).dilate); // expand

	Point center = conf->getCenterPoint();
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	contoursFrame = dilateFrame.clone(); // findCountours function modifies input frame !
	findContours(contoursFrame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // retrieves external contours

	positions.clear(); // start with empty list
	if( viewer ) {
		viewerContoursFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1); // clear contours previous frame
	}

	for( uint ii = 0; ii < contours.size(); ii++ ) {
		// make a bounding rectangle around each contour to calculate the angle
		objectBoundingRectangle = boundingRect(contours.at(ii));

		double x = objectBoundingRectangle.x - center.x + objectBoundingRectangle.width / 2;
		double y = objectBoundingRectangle.y - center.y + objectBoundingRectangle.height / 2;

		// count the amount of ball pixels in the rectangular area around each contour
		Mat ballBoxFrame = inRangeFrame(Rect(objectBoundingRectangle.tl(), objectBoundingRectangle.br())); // get the region of interest
		int pixels = countNonZero(ballBoxFrame);

		// check if size of ball is good enough
		if( pixels > conf->getBall(type).size ) {
			// The center of the ball is about 10cm above the ground and the camera mirror is about 80cm above the ground.
			// From the camera point of view it is more accurate to use the first ball pixel instead of the center of the object.
			// Search for the closest ball pixel.
			double radiusTan = conf->getCameraRadiusBallMax(); // initialize with number that will be out of range
			for( uint kk = 0; kk < contours.at(ii).size(); kk += 5 ) { // checking only 1 out of 5 will be good enough
				int xRadius = contours.at(ii).at(kk).x - center.x;
				int yRadius = contours.at(ii).at(kk).y - center.y;
				double radius = sqrt(pow(xRadius, 2) + pow(yRadius, 2));
				if( radius < radiusTan ) {
					radiusTan = radius;
				}
			}
			// reject all balls at the side of the mirror (color distortion) and in the tube reflection
			if( radiusTan < conf->getCameraRadiusBallMax() ) {
				ballSt ball; // create new object, which can be pushed to the list of ball positions
				ball.size = pixels;
				ball.radiusTan = radiusTan; // distance in round viewer pixels from center robot to nearest edge of ball
				ball.radiusLin = conf->calcDeWarp(ball.radiusTan); // distance in rectangular field pixels (1 pixel = 2cm) from center robot to nearest edge ball

				// calculate angle of ball to robot shooter
				// start at y-axis (= 0 degrees = shooter) instead of x-axis in distorted image
				ball.angle = (360.0f / (2.0f * CV_PI)) * atan(-x / -y); // upper part floor is lower y
				if( y >= 0.0f ) {
					ball.angle = ball.angle + 180.0f;
				} // atan has range of -90 to 90 degrees
				if( ball.angle < 0.0f ) {
					ball.angle = 360.0f + ball.angle;
				} // correct the -90 to 0 degrees to 270 to 360 degrees
				positions.push_back(ball);

				// cout << " ii " << ii << " size " << ball.size << " x " << x << " y " << y << " angle " << ball.angle << " radiusTan " << ball.radiusTan << endl;
				if( viewer ) {
					drawContours(viewerContoursFrame, contours, ii, 255, 1, 8, hierarchy, 0, Point());
				}
			}
		}
	}

	// sort on size
	sort(positions.begin(), positions.end());

	exportMutex.unlock();

	busy = false;
}

vector<ballSt> ballDetection::getPositions() {
	exportMutex.lock();
	std::vector<ballSt> retVal = positions;
	exportMutex.unlock();
	return retVal;
}

vector<ballSt> ballDetection::getPositionsExport() {
	vector<ballSt> retVal = getPositions();
	for( size_t ii = 0; ii < retVal.size(); ii++ ) {
		double tmpAngle = retVal[ii].angle + conf->getExportOffset().rzBall;
		// normalize to prevent issues when running of range for remote viewer export
		retVal[ii].angle = fmod(360.0 + tmpAngle, 360.0);
	}
	return retVal;
}

cv::Mat ballDetection::getContoursFrame() {
	exportMutex.lock();
	cv::Mat retVal = contoursFrame.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat ballDetection::getDilateFrame() {
	exportMutex.lock();
	cv::Mat retVal = dilateFrame.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat ballDetection::getErodeFrame() {
	exportMutex.lock();
	cv::Mat retVal = erodeFrame.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat ballDetection::getInRangeFrame() {
	exportMutex.lock();
	cv::Mat retVal = inRangeFrame.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat ballDetection::getViewerContoursFrame() {
	exportMutex.lock();
	cv::Mat retVal = viewerContoursFrame.clone();
	exportMutex.unlock();
	return retVal;
}
