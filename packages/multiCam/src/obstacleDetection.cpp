 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "obstacleDetection.hpp"
#include <algorithm>
#ifndef NOROS
#include "FalconsCommon.h"
#endif

using namespace std;
using namespace cv;

bool sortObstacleOnRadius(obstacleSt i, obstacleSt j) { return i.radiusLin > j.radiusLin; }

obstacleDetection::obstacleDetection(configurator *conf, preprocessor *prep) {
	this->conf = conf;
	this->prep = prep;

	exportMutex.lock();
	// initialize frames so even without running the update method the frames can be used by the viewer
	obstacleFrame = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC1);
	obstacleErodeFrame = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC1);
	obstacleDilateFrame = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC1);
	obstacleMask = Mat::ones(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC1);
	blackFloor = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC1);

	// make the result list empty so the first request does not get bogus data
	positions.clear();
	positionsExport.clear();
	exportMutex.unlock();

	cameraRadiusObstacleMinPrevious = -1; // be sure the mask is initialized during the first update
	cameraRadiusObstacleMaxPrevious = -1;
	centerPointPrevios = Point(-1,-1);
	busy = false;
}

// search the frame for obstacles
// regarding the MSL rules the obstacles should be black
// first we perform color filtering in the HSV image to search for black objects
// but we have to filter out some sections:
// - the robot itself is black, which is visible in the center of the image
// - furthermore near the edge and outside the mirror some black spots can occur
// - and last the keeper frame for robot 1 has to filtered out
// after the filtering the interesting black pixels are now available
// use the erode and dilate functions to remove noise and group sections (probably one obstacle)
// The openCv findContours function is used to find the contour points around these groups.
// Contour points are all points around the object where the angle of the line around the object changes
// a square has 4 contour points, but the dilate objects in the frame will have a weird figure, so the
// contain a lot more contour points e.g. 20
// to determine the angle of the objects, the center of a bounding box is used
// to determine the distance of the object, the closest contour point is used
// the size is calculated for the width and height of the bounding box and is used to remove noise (small objects)
// because the erode and dilate do not work perfectly it is possible that one obstacle is represented
// by multiple objects
// those will be merged by merging objects with more or less the same angle
// the angle of the merged object will be calculated from the merged x and y values
// the distance to the merged object will be shortest distance of the merged distances
void obstacleDetection::update( ) {

#ifdef NONO
	// check if the slider of the center point has been changed
	if( centerPointPrevios != conf->getCenterPoint() ||
			cameraRadiusObstacleMaxPrevious != conf->getCameraRadiusObstacleMin() ||
			cameraRadiusObstacleMinPrevious != conf->getCameraRadiusObstacleMax() ) {
		centerPointPrevios = conf->getCenterPoint();
		cameraRadiusObstacleMinPrevious = conf->getCameraRadiusObstacleMin();
		cameraRadiusObstacleMaxPrevious = conf->getCameraRadiusObstacleMax();

		// the edge of the mirror and outside the mirror dark objects might appear, update mask to update these areas
		obstacleMask = Mat::ones( conf->getViewPixels(), conf->getViewPixels(), CV_8UC1 ); // add all pixels to the mask (all pixels invalid)
		// remove all pixels within the edge of the mirror from the mask (make all these pixels valid again)
		circle( obstacleMask, conf->getCenterPoint(), conf->getCameraRadiusObstacleMax(), 0, -1, 8 , 0 );

		// the robot itself is black, so also seen as obstacle, add it to the exclude area for the obstacles
		circle( obstacleMask, conf->getCenterPoint(), conf->getCameraRadiusObstacleMin(), 1, -1, 8 , 0 ); // add inner pixel to the mask (invalidate)

		// the frame of the keeper is black, so also seen as obstacle, add these pixels to the mask (invalidate)
		if( robotId == 1 ) { // only for keeper
			int x = conf->getCenterPoint().x;
			int y = conf->getCenterPoint().y;
			rectangle( obstacleMask, Point(x-120,y-30), Point(x+120,y+20), 1, -1, 8 , 0); // mask mounting bracket keeper frame
			rectangle( obstacleMask, Point(0,y-20), Point(conf->getViewPixels(),y+15), 1, -1, 8 , 0); // keeper frame
			rectangle( obstacleMask, Point(conf->getViewPixels()-100,y-35), Point(conf->getViewPixels(),y-20), 1, -1, 8 , 0); // frame at right side (not 100% x-axis)
			rectangle( obstacleMask, Point(0,y-25), Point(50,y-20), 1, -1, 8 , 0); // frame at left side (not 100% x-axis)
		}
		// now we have a complete mask for objects that has ones for pixels that should be used and zeros for pixels that should be ignored
	}

	Point center = conf->getCenterPoint();
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<obstacleSt> allObst;

	// start with magic, first filter out the black pixels
	Scalar obstacleMin(conf->getObstacle().hue.min, conf->getObstacle().sat.min, conf->getObstacle().val.min);
	Scalar obstacleMax(conf->getObstacle().hue.max, conf->getObstacle().sat.max, conf->getObstacle().val.max);
	exportMutex.lock();
	inRange(prep->getHsv(), obstacleMin, obstacleMax, obstacleFrame); // keep only the black pixels

	// now apply the mask to invalidate all pixels that are not relevant
	blackFloor.copyTo( obstacleFrame, obstacleMask );

	// remove the noise and group the black pixels
	erode(obstacleFrame, obstacleErodeFrame, conf->getObstacle().erode);
	dilate(obstacleErodeFrame, obstacleDilateFrame, conf->getObstacle().dilate);
	exportMutex.unlock();

	// search for the contours in the grouped black pixels
	Mat contourFrame = obstacleDilateFrame.clone(); // findCountours function modifies input frame !
	findContours(contourFrame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // retrieves external contours

	// search:
	// get for each object the angle, distance and x,y values
	for (uint ii = 0; ii < contours.size(); ii++) {
		// make new obstacle to gather all information
		obstacleSt obst;

		// make a bounding rectangle around each contour to calculate the angle
		objectBoundingRectangle = boundingRect( contours.at(ii) );

		// get x and y values related to robot center
		obst.xLeft = objectBoundingRectangle.x - center.x;
		obst.xRight = objectBoundingRectangle.x + objectBoundingRectangle.width - center.x;
		obst.yTop = objectBoundingRectangle.y - center.y;
		obst.yBottom = objectBoundingRectangle.y + objectBoundingRectangle.height - center.y;
		obst.size =  contourArea(contours.at(ii));

		// the obstacle can be large, for best distance indication, use the nearest point to the obstacle
		// get this nearest point from the contours
		double radiusTan = conf->getViewPixels(); // initialize with number that will be out of range
		for (uint kk = 0; kk < contours.at(ii).size(); kk++) {
			int xRadius = contours.at(ii).at(kk).x - center.x;
			int yRadius = contours.at(ii).at(kk).y - center.y;
			double radius = sqrt( pow(xRadius,2) + pow(yRadius,2) );
			if( radius < radiusTan ) { radiusTan = radius; }
		}
		obst.radiusTan = radiusTan; // distance in round viewer pixels from center robot to nearest edge obstacle
		obst.radiusLin = conf->calcDeWarp( obst.radiusTan ); // distance in rectangular field pixels (1 pixel = 2cm) from center robot to neareset edge obstacle

		// calculate angle of this object
		// this angle is only used for merging, the final angle is calculated from the merged x and y values
		double x = ( obst.xLeft + obst.xRight ) / 2; // get center x of this object
		double y = ( obst.yTop + obst.yBottom) / 2; // get center y of this object

		// start at y-axis (= 0 degrees = shooter) instead of x-axis in distorted image
		double angle = ( 360.0f / (2.0f * CV_PI) ) * atan(-x/-y); // upper part floor is lower y
		if( y >= 0.0f ) { angle = angle + 180.0f; } // atan has range of -90 to 90 degrees
		if( angle < 0.0f ) { angle = 360.0f + angle; } // correct the -90 to 0 degrees to 270 to 360 degrees
		obst.angle = angle;

		allObst.push_back(obst);
	}

	// merge:
	// the optical system causes an obstacle to be stretched, generating a very high obstacle
	// when the obstacle is not fully black or there are reflections, the erode dilate functions
	// might not stitch all pixels together, so the findContours will create multiple objects
	// of this single obstacle
	// however we know if these objects are more or less at the same angle, they will be of the
	// same obstacle
	// try to combine objects that are roughly on the same angle
	for (uint ii = 0; ii < allObst.size(); ii++)
	{
		for (uint jj = 0; jj < allObst.size(); jj++)
		{
			if( ii != jj && allObst[ii].size > 0 && allObst[jj].size > 0 ) {
				float angleIi = allObst[ii].angle;
				float angleJj = allObst[jj].angle;
				float angleDiff = (abs)(angleJj - angleIi);
				// APOX todo: check if 8 degrees is a good value to decide if the objects belong to the same obstacle
				if( angleDiff < 8.0f ) {
					// merge ii and jj
					allObst[ii].xLeft = min(allObst[ii].xLeft, allObst[jj].xLeft);
					allObst[ii].xRight = max(allObst[ii].xRight, allObst[jj].xRight);
					allObst[ii].yTop = min(allObst[ii].yTop, allObst[jj].yTop);
					allObst[ii].yBottom = max(allObst[ii].yBottom, allObst[jj].yBottom);
					// we want to keep the closest radius to obstacle
					allObst[ii].radiusTan = min(allObst[ii].radiusTan, allObst[jj].radiusTan);
					allObst[ii].radiusLin = min(allObst[ii].radiusLin, allObst[jj].radiusLin);
					allObst[ii].size = allObst[ii].size + allObst[jj].size;
					allObst[ii].angle = (allObst[ii].angle + allObst[jj].angle)/2.0f;

					// jj should not be used anymore because it's data has been merged with ii
					allObst[jj].size = 0;
				}
			}
		}
	}

	// cleanup list of obstacles:
	positions.clear(); // start with empty list
	bool notify = false;
	// store all values that are good enough for usage by Ros or Viewer
	// the obstacle might have been merged from multiple objects, for accuracy recalculate the angle from the merged x,y values
	for (uint ii = 0; ii < allObst.size(); ii++)
	{
		if( allObst[ii].size > 0 )
		{
			// re-calculate angle of this obstacle
			double x = (allObst[ii].xLeft + allObst[ii].xRight)/2; // get center x of this obstacle
			double y = (allObst[ii].yTop + allObst[ii].yBottom)/2; // get center y of this obstacle

			// start at y-axis (= 0 degrees = shooter) instead of x-axis in distorted image
			double angle = ( 360.0f / (2.0f * CV_PI) ) * atan(-x/-y); // upper part floor is lower y
			if( y >= 0.0f ) { angle = angle + 180.0f; } // atan has range of -90 to 90 degrees
			if( angle < 0.0f ) { angle = 360.0f + angle; } // correct the -90 to 0 degrees to 270 to 360 degrees

			allObst[ii].angle = angle; // store the new angle
			positions.push_back(allObst[ii]);

			// Sort the obstacles on radius
			std::sort(positions.begin(), positions.end(), sortObstacleOnRadius);

			// only if one or more obstacles are large enough send information to ros
			if( allObst[ii].size > conf->getObstacle().size )
			{
				notify = true;
			}
#ifndef NOROS
			else
			{
				TRACE("Found obstacle with insufficient size %d", allObst[ii].size);
			}
#endif
		}
	}
	exportMutex.lock();
	positionsExport = positions;
	exportMutex.unlock();

	if( notify ) {
		// Notify new ball position
		notifyNewPos();
	}
#endif
	busy = false;
}

vector<obstacleSt> obstacleDetection::getPositions() {
	exportMutex.lock();
	vector<obstacleSt> retVal = positionsExport;
	exportMutex.unlock();
	return retVal;
}

vector<obstacleSt> obstacleDetection::getPositionsExport() {
	vector<obstacleSt> retVal = getPositions();
	for( size_t ii = 0; ii < retVal.size(); ii++ ) {
		double tmpAngle = retVal[ii].angle + conf->getExportOffset().rzObstacle;
		// normalize to prevent issues when running of range for remote viewer export
		retVal[ii].angle = fmod(360.0 + tmpAngle, 360.0);
	}
	return retVal;
}

cv::Mat obstacleDetection::getObstacle() {
	exportMutex.lock();
	cv::Mat retVal = obstacleFrame.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat obstacleDetection::getObstacleErode() {
	exportMutex.lock();
	cv::Mat retVal = obstacleErodeFrame.clone();
	exportMutex.unlock();
	return retVal;
}
cv::Mat obstacleDetection::getObstacleDilate() {
	exportMutex.lock();
	cv::Mat retVal = obstacleDilateFrame.clone();
	exportMutex.unlock();
	return retVal;
}

// below only for interfacing with Ros, no obstacle detection functionality

void obstacleDetection::notifyNewPos()
{
#ifndef NOROS
	std::vector<obstaclePositionType> obstacles;

	vector<obstacleSt> exportPos = getPositionsExport( );

	for (uint ii = 0; ii < exportPos.size(); ii++)
	{
		double size = exportPos[ii].size;
		// Note: the world model needs to discard obstacles that are outside the field e.g. the black borders around the floor
		// this is depending on the final position, which is determined by the world model instead of the localization on this robot
		if( size > conf->getBall(obstacleType).pixels )  {
			double angleDeg =  fmod(exportPos[ii].angle, 360.0);
			double angleRad = (2.0f * M_PI / 360.0f) * angleDeg; // counter clockwise angle between this robot shooter and the obstacle
			double radiusLin = 0.02f * exportPos[ii].radiusLin; // from center of this robot to closest edge of obstacle, to get the center of the obstacle add obstacle radius e.g. 0.25 meter (TODO!)
			double score = size/15000.0f;
			// APOX todo: set to magenta or cyan when detected
			int color = 0; // 0 is black
			if( score > 1.0 ) { score = 1.0; }
			// cout << "obstacle index " << ii << " size " << size << " score " << score << " angle degrees " << angleDeg << " angle radians " << angleRad << " radiusLin " << radiusLin << endl;

			obstaclePositionType obstacle;

			obstacle.setAngle(angleRad);
			obstacle.setRadius(radiusLin);
			obstacle.setConfidence(score);
			obstacle.setColor(color);

			obstacles.push_back(obstacle);
		}
	}

	for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); ++iter)
	{
		if (*iter != NULL)
		{
			(*iter)->update_own_obstacle_position(obstacles, conf->getObstacleLatencyOffset());
		}
	}
#endif
}

void obstacleDetection::attach(observer *observer)
{
    vecObservers.push_back(observer);
}

void obstacleDetection::detach(observer *observer)
{
    vecObservers.erase(std::remove(vecObservers.begin(), vecObservers.end(), observer), vecObservers.end());
}
