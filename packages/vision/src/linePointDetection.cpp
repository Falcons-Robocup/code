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

// The goal of the Line Point Detection is to create points from the white lines
// on the floor of the soccer field.
//
// Later on these points are used to determine the robot position on the soccer floor.
//
// The line point detection uses spiral lines which run from the center to the outside.
// A line point is created when the spiral line crosses a line and a number of conditions are met.
//
// The main challenge of the line point detection is how to handle sun spots, goal and ball reflections.
// Depending on the windows the sun can also create lines on the floor.
// Note: the preprocessor already contains a function to block large sun spots, but the
// preprocessor is not able to remove lines created by the sun.
// However if the lines are wider then the normal lines then the line point detector is
// able to distinguish them from the real lines.
//
// This is been performed by only accepting small lines, because the spiral line
// can partly run in parallel with the field line, which then is ignored because of
// the length, the spiral line is running clockwise and counter clockwise, so at
// least one of then will create a valid line point.
// This allows to set the line width very tight which makes it possible to filter
// the sun lines in most conditions.


#include "linePointDetection.hpp"
#include <math.h> // atan2
#include <stdio.h> // printf

// amount of steps in the 360 degrees circle
#define NRANGLES 100

using namespace cv;
using namespace std;

linePointDetection::linePointDetection(configurator *conf, preprocessor *prep) {
	this->conf = conf;
	this->prep = prep;

	centerPointPrevios = Point(-1,-1); // force generating the mask the first time the findPointsInFrame is performed
	cameraRadiusLineMaxPrevious = -1;

	goodEnoughLoc.lastActive = INT_MAX;
	goodEnoughLoc.goodEnough = false;
	goodEnoughLoc.score = 1.0;


	exportMutex.lock();
	lineUnfilteredFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	lineErodeFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	lineDilateFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	lineFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	lineFrameExport = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	sunSpotFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	sunSpotContoursFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	lineBallMask = Mat::ones( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );

	linePointsExport.clear();
	linePointsAllExport.clear();
	linePointsRadianMeterExport.clear();
	linePointsRejectExport.clear();

	int centerPixel = conf->getCenterPixel();

	for( int y = -conf->getCameraRadiusLineMax(); y < conf->getCameraRadiusLineMax(); y += 10 ) { // y region of interest is from -radius (upper section of field) to +radius
		// for every y line calculate multiple angles, e.g. 0 degrees, 60 and 120 degrees
		const int diffAngles = 3;
		std::vector<cv::Point> spiralLine[diffAngles]; // store for each different angle line all pixels
		for( int x = -conf->getCameraRadiusLineMax(); x < conf->getCameraRadiusLineMax(); x++ ) { // x region of interest is from -radius (left) to +radius (right)
			double radius = sqrt( pow(x,2) + pow(y,2) );
			if( radius < conf->getCameraRadiusLineMax() ) {
				// radius within in range for particular robot, create the pixels for all angle lines
				double angle = atan2(-y,x); // negative y is upper part field
				for( int kk = 0; kk < diffAngles; kk++ ) {
					int x2 = round(radius * cos(angle + kk*CV_PI/diffAngles));
					int y2 = round(radius * sin(angle + kk*CV_PI/diffAngles));
					if( x2 > -centerPixel && x2 < centerPixel && y2 > -centerPixel && y2 < centerPixel ) {
						spiralLine[kk].push_back(conf->getCenterPoint() + Point(x2,y2));
					}
				}
			}
		}
		// Store the new created points of a complete spiral line.
		for(int kk = 0; kk < diffAngles; kk++ ){
			spiralLines.push_back(spiralLine[kk]);
		}
	}

	size_t spiralPixels = 0;
	for( size_t jj = 0; jj<spiralLines.size(); jj++ ) {
		spiralPixels += spiralLines[jj].size();
	}
	printf("INFO      : Amount of spirals: %lu which have a total of: %lu spiral points\n", spiralLines.size(), spiralPixels );

	spiralLineFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 );
	for( size_t jj = 0; jj<spiralLines.size(); jj++ ) {
		for( size_t ii = 0; ii < spiralLines[jj].size(); ii++ ) {
			line( spiralLineFrame, spiralLines[jj].at(ii), spiralLines[jj].at(ii), 255, 1, 8 , 0 );
		}
	}
	exportMutex.unlock();
}

// Show line points in the distorted round viewer
// In the distorted viewer the positive y-axis is 0 degrees (and positive x-axis is 270 degrees)
// The angle is anti clock wise
Mat linePointDetection::getLinePointsFrame( ){
	Mat frame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	exportMutex.lock();
	for ( size_t ii = 0; ii < linePointsExport.size(); ii++ ) {
		line( frame, linePointsExport.at(ii), linePointsExport.at(ii), 255, 4, 8);
	}
	exportMutex.unlock();
	return frame;
}

// Show all found white pixels on grid lines as small grey lines in the round viewer
// In the distorted viewer the positive y-axis is 0 degrees (and positive x-axis is 270 degrees)
// The angle is anti clock wise
Mat linePointDetection::getLinePointsAllFrame( ){
	Mat frame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	exportMutex.lock();
	for ( size_t ii = 0; ii < linePointsAllExport.size(); ii++ ) {
		line( frame, linePointsAllExport.at(ii), linePointsAllExport.at(ii), 255, 1, 8);
	}
	exportMutex.unlock();
	return frame;
}

// Show all rejected line points in the distorted round viewer
// In the distorted viewer the positive y-axis is 0 degrees (and positive x-axis is 270 degrees)
// The angle is anti clock wise
Mat linePointDetection::getLinePointsRejectFrame( ){
	Mat frame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC1);
	exportMutex.lock();
	for ( size_t ii = 0; ii < linePointsRejectExport.size(); ii++ ) {
		line( frame, linePointsRejectExport.at(ii), linePointsRejectExport.at(ii), 255, 4, 8);
	}
	exportMutex.unlock();
	return frame;
}

vector<angleRadiusSt> linePointDetection::getLinePointsRadianMeter() {
	exportMutex.lock();
	vector<angleRadiusSt> retVal = linePointsRadianMeterExport;
	exportMutex.unlock();
	return retVal;
}
Mat linePointDetection::getSpiralLineFrame() {
	exportMutex.lock();
	cv::Mat retVal = spiralLineFrame.clone();
	exportMutex.unlock();
	return retVal;
}

Mat linePointDetection::getSunSpotContours() {
	exportMutex.lock();
	cv::Mat retVal = sunSpotContoursFrame.clone();
	exportMutex.unlock();
	return retVal;
}

Mat linePointDetection::getLineUnfiltered() {
	exportMutex.lock();
	cv::Mat retVal = lineUnfilteredFrame.clone();
	exportMutex.unlock();
	return retVal;
}

Mat linePointDetection::getLineErode() {
	exportMutex.lock();
	cv::Mat retVal = lineErodeFrame.clone();
	exportMutex.unlock();
	return retVal;
}

Mat linePointDetection::getLineDilate() {
	exportMutex.lock();
	cv::Mat retVal = lineDilateFrame.clone();
	exportMutex.unlock();
	return retVal;
}

Mat linePointDetection::getSunSpot() {
	exportMutex.lock();
	cv::Mat retVal = sunSpotFrame.clone();
	exportMutex.unlock();
	return retVal;
}

Mat linePointDetection::getLine() {
	exportMutex.lock();
	cv::Mat retVal = lineFrameExport.clone();
	exportMutex.unlock();
	return retVal;
}

// Convert the absolute x and y position of the points to an angle in radians and radius in meters
// The angle is anti clock wise
// In the round viewer the location of the shooter is 0 degrees, this is at top upper y-axis
// In the rectangular viewer the right x-axis is 0 degrees.
// Note: this is different then the MSL specifications, which use a portrait layout.
void linePointDetection::updateLinePointsRadianMeter() {
	linePointsRadianMeter.clear();
	for( size_t ii = 0; ii < linePoints.size(); ii++ ) {
		int xRelative = linePoints.at(ii).x - conf->getCenterPoint().x;
		int yRelative = linePoints.at(ii).y - conf->getCenterPoint().y;

		double xMetal = xRelative + conf->getExportOffset().x; // in pixels, directions calibration sliders depends on orientation on field
		double yMetal = yRelative + conf->getExportOffset().y; // in pixels
		double angleMetal =  atan2(-yMetal,xMetal) + 2.0*CV_PI * conf->getExportOffset().rz/360.0; // in radians

		// APOX todo: add note why the -90 degrees is required
		angleRadiusSt radMet;
		radMet.angle = - 2.0*CV_PI * 90.0f/360.0 + angleMetal;
		radMet.radius = conf->calcDeWarp( sqrt( pow(xMetal,2) + pow(yMetal,2) ) );
		linePointsRadianMeter.push_back(radMet);
	}
}

// Store the center of the first and last found point if the line is not to long.
void linePointDetection::storePoint(Point firstPoint, Point lastPoint) {
	// Determine center of first and last point on the current grid line
	Point averagePoint = Point( (firstPoint.x + lastPoint.x)/2.0, (firstPoint.y + lastPoint.y)/2.0);

	// if the line is to long then it probably not a field line
	double lineWidthPow2 = pow(firstPoint.x - lastPoint.x, 2) + pow(firstPoint.y - lastPoint.y, 2);
	double lineRejectWidthPow2 = pow(conf->getLine().lineRejectWidth, 2);
	if( lineWidthPow2 < lineRejectWidthPow2 ) {
		linePoints.push_back( averagePoint );
	} else {
		// store to visualize
		linePointsReject.push_back( averagePoint );
	}
}

// Get the points of each spiral line.
// The points run from the center to the outside
// When the spiral crosses the line, get the first point where the line was touched and
// the last point where the line was touched. These two points are used to determine
// the center of both points.
void linePointDetection::update( ) {
	// if value changed update mask to remove line pixels outside or near edge mirror
	if( centerPointPrevios != conf->getCenterPoint() || cameraRadiusLineMaxPrevious != conf->getCameraRadiusLineMax() ) {
		centerPointPrevios = conf->getCenterPoint();
		cameraRadiusLineMaxPrevious = conf->getCameraRadiusLineMax();

		// to reduce processing power, only update when needed (adjusted configuration)

		// outside the round camera view, images are reflected, exclude these for balls and lines
		lineBallMask = Mat::ones( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 ); // add all pixels to the mask (all pixels invalid)
		// remove all pixels within the edge of the mirror from the mask (make all these pixels valid again)
		circle( lineBallMask, conf->getCenterPoint(), conf->getCameraRadiusBallMax(), 0, -1, 8 , 0 );

		// TODO: checkout why this code mask not used anymore
	}

	// ## line point detection filtering ##
	int valMin;
	if( conf->getCalibrateMode() ) {
		// calibration mode uses black and white chessboard floor which requires a higher minimal value
		valMin = conf->getLine().calVal.min;
	} else {
		valMin = conf->getLine().val.min;
	}
	Scalar lineMin(conf->getLine().hue.min, conf->getLine().sat.min, valMin);
	Scalar lineMax(conf->getLine().hue.max, conf->getLine().sat.max, conf->getLine().val.max);
	exportMutex.lock();
	inRange(prep->getHsv(), lineMin, lineMax, lineFrame); // filter line out

	lineFrame.copyTo(lineUnfilteredFrame);

	// try to detect large white spots, these are probably unwanted sun spots
	erode(lineFrame, lineErodeFrame, conf->getLine().erode);
	dilate(lineErodeFrame, lineDilateFrame, conf->getLine().dilate);
	detectSunSpot(); // after this function sun spots available in sunSpotFrame

	// remove the bright sun spot from the line point detection to prevent white spots because bright sun light or light reflection
	lineFrame = lineFrame - sunSpotFrame;

	// the goalRearMask has a non zero value for robot1, and is zero for all other robots
	// this filter has been added to mask the net of the goal, which generate quite some white pixels
	// which causes a reduction in the use of the good line pixels
	// to prevent issues with localization for robot1 the filter only works if robot one is close to the
	// goal line and if the angle is near 0 degrees or 180 degrees (depending on the side of the field)
	int goalBackDistance = conf->getLine().goalRearDistance;
	int xRight = conf->getFloorSize().xRight;
	int goalBackDistanceRight = xRight - goalBackDistance;
	// printf("goalmask: lastActive %4d, x %3.0f, angle %3.0f, angleOffset %3d, watchdog %3d, rear distance %3d\n",
	//		goodEnoughLoc.lastActive, goodEnoughLoc.pos.x, goodEnoughLoc.pos.rz, conf->getLine().goalRearAngle, conf->getLine().goalRearWatchdog, conf->getLine().goalRearDistance);
	if( goodEnoughLoc.lastActive < conf->getLine().goalRearWatchdog ) {
		if( ( goodEnoughLoc.pos.x < goalBackDistance ) &&
				( ( goodEnoughLoc.pos.rz < conf->getLine().goalRearAngle ) ||
				( goodEnoughLoc.pos.rz > ( 360 - conf->getLine().goalRearAngle ) ) ) ) {
			// the pos.x is in "meters" while the xMask is in "pixels", TODO: add correction based on deWarp
			int xMask = conf->getViewPixels() - conf->getLine().goalRearMask + goodEnoughLoc.pos.x - 50;
			rectangle(lineFrame, Point(0,xMask), Point(conf->getViewPixels(),conf->getViewPixels()), 0, -1);
		} else if( ( goodEnoughLoc.pos.x > goalBackDistanceRight ) &&
				( goodEnoughLoc.pos.rz > ( 180 - conf->getLine().goalRearAngle ) ) &&
				( goodEnoughLoc.pos.rz < ( 180 + conf->getLine().goalRearAngle ) ) ) {
			// the pos.x is in "meters" while the xMask is in "pixels", TODO: add correction based on deWarp
			int xMask = conf->getViewPixels() - conf->getLine().goalRearMask + ( xRight - goodEnoughLoc.pos.x ) - 50;
			rectangle(lineFrame, Point(0,xMask), Point(conf->getViewPixels(),conf->getViewPixels()), 0, -1);
		}
	}

	linePixels = countNonZero( lineFrame );
	lineFrame.copyTo(lineFrameExport);
	exportMutex.unlock();

	// inLinePixelsList.clear();
	linePoints.clear();
	linePointsAll.clear();
	linePointsReject.clear();

	for( size_t jj = 0; jj<spiralLines.size(); jj++ ) {
		// new spiral
		bool found = false;
		int offTheLineCounter = 0; // used for noise reduction
		int onTheLineCounter = 0; // used as lower threshold to filter single onLine pixels
		Point firstPoint = Point(0,0);
		Point lastPoint = Point(0,0);

		for( size_t ii = 0; ii < spiralLines[jj].size(); ii++ ) {
			bool onTheLine = lineFrame.at<uchar>(spiralLines[jj].at(ii)) != 0;
			if( onTheLine ) { linePointsAll.push_back(spiralLines[jj].at(ii)); } // show all found white pixels on grid lines as small grey lines in the round viewer

			if( found ) {
				// search for end of line
				if( onTheLine )	{
					lastPoint = spiralLines[jj].at(ii);
					onTheLineCounter++;
					offTheLineCounter = 0;
				} else {
					offTheLineCounter++;
				}

				if( offTheLineCounter > conf->getLine().lineHoleWidth || ii == (spiralLines[jj].size()-1) ) {
					// we did not see a point on the line for a while
					// or this was the last point of the spiral
					if( onTheLineCounter > conf->getLine().linePointsMinimal ) {
						// with less then 5 pixels on the line it is classified as noise
						storePoint(firstPoint, lastPoint);
					}
					found = false;
				}
			} else {
				// search for begin of line
				if( onTheLine ) {
					firstPoint = spiralLines[jj].at(ii);
					lastPoint = spiralLines[jj].at(ii); // initialize in case not another point is found in this found state
					found = true;
					onTheLineCounter = 1;
					offTheLineCounter = 0;
				}
			}
		}
	}

	// the search algorithm has a maximum on the amount of used points (yaml: linePointsNumber)
	// in case more line points are available then this maximum, randomize the order so the algorithm uses
	// a homogeneous section of the field
	if( linePoints.size() > conf->getLine().linePointsNumber ) {
		random_shuffle(linePoints.begin(),linePoints.end());
	}

	updateLinePointsRadianMeter(); // update for determine position function

	exportMutex.lock();
	linePointsExport = linePoints;
	linePointsAllExport = linePointsAll;
	linePointsRadianMeterExport = linePointsRadianMeter;
	linePointsRejectExport = linePointsReject;
	exportMutex.unlock();
}

/* chessboard calibration
 * For chesboard calibration we do not search for line points but for edges between the white and black fields
 */
void linePointDetection::updateEdge() {
	// if value changed update mask to remove line pixels outside or near edge mirror
	if( centerPointPrevios != conf->getCenterPoint() || cameraRadiusLineMaxPrevious != conf->getCameraRadiusLineMax() ) {
		centerPointPrevios = conf->getCenterPoint();
		cameraRadiusLineMaxPrevious = conf->getCameraRadiusLineMax();

		// to reduce processing power, only update when needed (adjusted configuration)

		// outside the round camera view, images are reflected, exclude these for balls and lines
		lineBallMask = Mat::ones( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 ); // add all pixels to the mask (all pixels invalid)
		// remove all pixels within the edge of the mirror from the mask (make all these pixels valid again)
		circle( lineBallMask, conf->getCenterPoint(), conf->getCameraRadiusBallMax(), 0, -1, 8 , 0 );
	}

	exportMutex.lock();
	inLinePixelsList.clear();
	for( int angleIndex = 0; angleIndex < NRANGLES; angleIndex++ ) {
		bool whiteField = false;
		bool storePixel;
		for( int radius = conf->getCameraRadiusObstacleMin(); radius < conf->getCameraRadiusLineMax(); radius++ ) {
			if( inLinePixelsList.size() < 100 ) {
				storePixel = false;
				// start at y-axis (= 0 degrees) instead of x-axis
				int xLocation = round( conf->getCenterPixel() - radius * sin(2.0*CV_PI*angleIndex/NRANGLES) );
				int yLocation = round( conf->getCenterPixel() - radius * cos(2.0*CV_PI*angleIndex/NRANGLES) ); // upper part view is lower y
				bool whitePixel = lineFrame.at<uchar>(yLocation,xLocation) == 255;
				if( whitePixel ) {
					// so this is a pixel in a white field
					if( whiteField) {
						// the previous pixel was also in a white field, do nothing
					} else {
						// the previous pixel was in a black field, edge detected
						storePixel = true;
					}
					whiteField = true;
				} else {
					// so this a pixel in a black field
					if( whiteField ) {
						// the previous pixel was in a white field, edge detect
						storePixel = true;
					} else {
						// the previous pixel was also in a black field, do nothing
					}
					whiteField = false;
				}
				if( storePixel ) {
					positionStFl store;
					store.radiusTan = radius;
					store.radiusLin = conf->calcDeWarp( radius );
					store.angle = 360.0f*angleIndex/NRANGLES;
					store.x = -1; // not used
					store.y = -1; // not used
					store.rz = -1; // not used
					inLinePixelsList.push_back(store);
				}
			}
		}
	}
	exportMutex.unlock();
}

// Search for areas that contain a lot of white pixels.
// This function has to distinguish between sun spots and white lines.
// White lines result in relative a small amount of white pixels per area, while
// sun spots result in large amount of white pixels.
// This function generates an image that can be used to block the sun spot
// and also an image is created that can be used in the viewer to show the contour
// of the sun spot in the round viewers.
void linePointDetection::detectSunSpot( ) {
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	vector<Vec4i> hierarchy;
	vector<vector<Point> > sunSpotContours;

	Mat contourFrame = lineDilateFrame.clone(); // findCountours function modifies input frame !
	findContours(contourFrame, sunSpotContours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // retrieves external contours

	sunSpotFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 ); // clear sun spots previous frame
	sunSpotContoursFrame = Mat::zeros( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 ); // clear contours previous frame

	for( size_t ii = 0; ii < sunSpotContours.size(); ii++) {
		// Get the bounding rectangle around each contour to get the width, height and use it to count the white pixels
		objectBoundingRectangle = boundingRect( sunSpotContours.at(ii) );

		int width = objectBoundingRectangle.width;
		int height = objectBoundingRectangle.height;

		// count the amount of sun pixels in the rectangular area around each contour
		Mat sunBoxFrame = lineFrame(Rect(objectBoundingRectangle.tl(),objectBoundingRectangle.br())); // get the region of interest
		int sunBoxPixels = countNonZero( sunBoxFrame );

		// the expected pixels is dependent of the width and height of the region of interest
		int requiredPixels = (int)( conf->getLine().sunSpotRatio * width * height / 100.0 );

		if( width > conf->getLine().sunSpotWidth && height > conf->getLine().sunSpotHeight && sunBoxPixels > requiredPixels ) {
			// cout << "white: ii " << ii << " width " << width << " height " << height << " size " << sunBoxPixels << " required " << requiredPixels << endl;
			drawContours( sunSpotFrame, sunSpotContours, ii, 255, CV_FILLED, 8, hierarchy, 0, Point() );
			drawContours( sunSpotContoursFrame, sunSpotContours, ii, 255, 1, 8, hierarchy, 0, Point() );
			// rectangle( sunSpotFrame, objectBoundingRectangle.tl(), objectBoundingRectangle.br(), 255, 1, 8, 0 );
			// sunBoxFrame.copyTo( sunSpotFrame.colRange(100, 100+sunBoxFrame.cols).rowRange(100,100+sunBoxFrame.rows));
		}
	}
}

