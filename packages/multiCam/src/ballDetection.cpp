 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "ballDetection.hpp"

#include <math.h>

// #define BALL_LATENCY_MEASUREMENT

using namespace std;
using namespace cv;

ballDetection::ballDetection(cameraReceive *camRecv, configurator *conf, deWarper *dewarp, preprocessor *prep,
		size_t type, size_t camIndex) {
	this->camRecv = camRecv;
	this->conf = conf;
	this->dewarp = dewarp;
	this->prep = prep;
	this->type = type; // ball detection re-used for cyan and magenta detection, type is used to distinguish
	this->camIndex = camIndex;

	height = conf->getCameraHeight();
	width = conf->getCameraWidth();

	exportMutex.lock();
	// initialize frames so even without running the update method the frames can be used by the viewer
	erodeFrame = Mat::zeros(height, width, CV_8UC1);
	dilateFrame = Mat::zeros(height, width, CV_8UC1);
	inRangeFrame = Mat::zeros(height, width, CV_8UC1);

	possessionFrame = Mat::zeros(height, width, CV_8UC1);
	possessionPixels = 0;

	// make the result list empty so the first request does not get bogus data
	positions.clear();
	positionsRemoteViewerExportMutex.unlock();
	ballPointListExportMutex.unlock();
	exportMutex.unlock();
	busy = false;
	printCount = 0;
	exportMutex.lock();
	for (size_t ii = 0; ii < height; ii++) { // create vector for the closest obstacle pixels
		closestPixels.push_back(-1);
		closestGroups.push_back(-1);
	}
	exportMutex.unlock();
}

void ballDetection::keepGoing() {
	while (1) {
		update();
	}
}

typedef enum closestState {
	CLOSEST_INIT = 0, CLOSEST_NOTHING, CLOSEST_FOUND, CLOSEST_PENDING, CLOSEST_STORE,
} closestStateT;

// try to find the closest line for obstacles
// first get all closest obstacle points
// then group them together
// use these groups to split the dilated obstacle image, so the borders are separated
// and even robot close to each other (but at a different distance)
// NOTE: the camera is rotated, so the nearby axis the y axis (x = 0)
void ballDetection::findClosestLineGroups() {
	// run through all lines
	for (size_t yy = 0; yy < height; yy++) {
		// get the closest pixel of this particular line
		// Note: the lock mutex is already set by the caller of this function
		closestPixels[yy] = -1; // default closest obstacle pixel not found
		for (size_t xx = 0; xx < width; xx++) {
			uchar pixelVal = inRangeFrame.at<uchar>(yy, xx);
			if (pixelVal != 0) {
				// found the obstacle pixel, store and goto next line
				closestPixels[yy] = xx;
				xx = width; // found it, go to next line
			}
		}
	}

	// now group the pixels, each group represents one obstacle

	// initialize the closestGroup vector, meaning the closest pixels are not related to any group
	for (size_t yy = 0; yy < height; yy++) {
		closestGroups[yy] = -1; // not assigned to any group
	}

	ssize_t groupIndex = 0;

	for (ssize_t yy0 = 0; yy0 < height - 1; yy0++) {
		ssize_t xx0 = closestPixels[yy0];
		if (xx0 >= 0) {
			if (closestGroups[yy0] < 0) {
				// this pixel was not yet related to a group, assign to new group
				closestGroups[yy0] = groupIndex;
				groupIndex++; // prepare for next group
			}

			// determine distance to all of the remaining pixels
			for (ssize_t yy1 = yy0 + 1; yy1 < height; yy1++) {
				ssize_t xx1 = closestPixels[yy1];
				if (xx1 >= 0) {
					double delta = sqrt(pow(xx0 - xx1, 2) + pow(yy0 - yy1, 2));
					double threshold = (xx1 - 470.0) / -16.0;
					if (threshold > 15.0) {
						threshold = 15.0;
					}
					if (threshold < 3.0) {
						threshold = 3.0;
					}
					if (delta < threshold) {
						// this obstacle pixels is close to the yy0 obstacle pixels, assign to same group
						closestGroups[yy1] = closestGroups[yy0];
					}
				}
			}
		}
	}
	// now we know for each closest pixel if it belongs to a group and if so to which group
}

// Search for balls in the related camera (each camera has it's own ball object)
// The balls are provided as "yellow" pixels in the camera image
// It is expected some false ball pixels will be provided
// These will be filtered by the Erode, Dilate and the minimum size
// The dewarp function is used to convert the ball camera pixels to x,y offset in mm
// The x,y position is relative to the robot (and camera on that robot)
// The camera is 90 degrees rotated, so the x mainly represents the distance between the
// robot and the ball, while the y mainly represents the angle
// An y of 0 means the ball is straight for the camera, and negative y means the ball
// is at the left side of the camera and a positive y, the ball is at the right
// side of the robot
// The x,y Cartesian coordinate converted to Polar coordinates, to make them
// easier to use as relative to the Robot
// Note: The radius is measured from the closest yellow pixel, but because of the camera position, this
// is roughly the same as the center of the ball.
void ballDetection::update() {
// get the pixels of the camera
	vector<linePointSt> cartesian;

// wait for points, delivered by the receiver
// Note: one point is actually a line (xBegin,y) and (xEnd,y)
// TODO: the clear of the getAndClear does not add any functionality because the thread blocks until
// new data is received, which means the data is anyway only used once
// but also means the notify to ros runs only once, so no problem over there
// but sending the ball list to the remote viewer runs as a different thread and might
// send the data multiple times, so for that purpose the list needs to be cleared
// TODO: apparently the same ghost ball is send multiple times to world model ???
	if (type == ballType) {
		cartesian = camRecv->getAndClearBallPointsWait(camIndex);
	} else if (type == ballFarType) {
		cartesian = camRecv->getAndClearBallFarPointsWait(camIndex);
	} else if (type == obstacleType) {
		cartesian = camRecv->getObstaclePointsWait(camIndex);
	} else {
		printf("ERROR     : ballDetection type %zu unknown, note: ballDetection also used for obstacleDetection\n",
				type);
		exit( EXIT_FAILURE);
	}
#ifdef BALL_LATENCY_MEASUREMENT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double startTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
#endif

	ballPointListExportMutex.lock();
	ballPointList = cartesian;
	ballPointListExportMutex.unlock();

// construct "image" from received points
	exportMutex.lock();
	inRangeFrame = Mat::zeros(height, width, CV_8UC1);
	for (size_t ii = 0; ii < cartesian.size(); ii++) {
		int xBegin = cartesian[ii].xBegin;
		if (xBegin < conf->getBall(type).distanceOffset) {
			xBegin = conf->getBall(type).distanceOffset;
		}
		int xEnd = cartesian[ii].xEnd;
		int y = cartesian[ii].yBegin;
		// TODO: use yEnd to create a rectangle instead of line
		// TODO: set line width back to 1 if all lines are scanned in grabber (now only half the lines used)
		if (xEnd >= xBegin) {
			// xEnd can be lower the xBegin because of using the distance offset (mainly required for obstacles because of the camera cover)
			line(inRangeFrame, Point(xBegin, y), Point(xEnd, y), 255, 2, 0);
		}
	}

// mask camera cover to prevent false obstacles (and fake balls)
// create two triangles that block the fake obstacles
// left triangle
	int blockColor = 0;
	Point blockTriangle[1][3];
	blockTriangle[0][0] = Point(0, 0);
	blockTriangle[0][1] = Point(0, conf->getMask().leftCloseby);
	blockTriangle[0][2] = Point(conf->getMask().leftFarAway, 0);

	const Point* pptLeft[1] = { blockTriangle[0] };
	int npt[] = { 3 };
	fillPoly(inRangeFrame, pptLeft, npt, 1, blockColor, 0);

// right triangle
	blockTriangle[0][0] = Point(0, ROI_HEIGHT - 1);
	blockTriangle[0][1] = Point(0, conf->getMask().rightCloseby);
	blockTriangle[0][2] = Point(conf->getMask().rightFarAway, ROI_HEIGHT - 1);

	const Point* pptRight[1] = { blockTriangle[0] };
	fillPoly(inRangeFrame, pptRight, npt, 1, blockColor, 0);

	// determine obstacle groups from the closest by obstacle pixels
	if (type == obstacleType) {
		findClosestLineGroups();
	}

// typically there are points of 1 pixel wide, likely caused by the demosaicing in the FPGA
	erode(inRangeFrame, erodeFrame, conf->getBall(type).erode);			// reduce
// now cluster together what is left
	dilate(erodeFrame, dilateFrame, conf->getBall(type).dilate);			// expand

	// use the closest by obstacle groups to separate the dilated frame, so the contours will not stretch
	// over multiple obstacles (e.g. black border)
	if (type == obstacleType) {
		ssize_t groupIndexPrev = closestGroups[0];
		for (ssize_t yy = 1; yy < height; yy++) {
			ssize_t groupIndex = closestGroups[yy];
			// add a blocking line when the group is changing or no obstacle pixel found on line
			if ((groupIndexPrev != groupIndex) || (groupIndex == -1)) {
				groupIndexPrev = groupIndex;
				// make the blocking line wider if the dilate is wider to remove dilated pixels on the other side of the blocking line
				int lineWidth = conf->getBall(type).dilateSlider;
				line(dilateFrame, Point(0, yy), Point(width - 1, yy), 0, lineWidth);
			}
		}
	}

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

// determine contours that will encapsulate the clusters
	Mat tmp;
	dilateFrame.copyTo(tmp);			// copy dilateFrame otherwise it will be modified by findContours
	findContours(tmp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	positions.clear();			// start with empty list
	for (size_t ii = 0; ii < contours.size(); ii++) {
		// make a bounding rectangle around each contour to calculate the distance, center and amount of ball pixels
		Rect rect = boundingRect(contours.at(ii));

		// determine the pixel closest by to the ball (radius)
		int xClosestBy = rect.x;

		// determine the center of the ball (azimuth)
		int yCenter = rect.y + rect.height / 2;

		// determine the amount of ball pixels inside the bounding rectangle
		int pixels = countNonZero(inRangeFrame(rect));

		// convert from Cartesian to Polar
		int16_t xField = 0;
		int16_t yField = 0;
		ballSt polar;

		// diagnostics
		bool accept = false;
		bool attemptedDewarp = false;
		bool usedDewarp = false;
		bool enoughPixels = pixels > conf->getBall(type).pixels;
		if (enoughPixels && (type == ballFarType)) {
			enoughPixels = pixels < conf->getBall(type).pixelsMax;
		}
		if (type == obstacleType) {
			// the height of an obstacle (robot) is larger then the width
			// however there are cases when 2 robots are next to each other
			// so only reject when obstacle is really low
			// this reject the black border lines
			if (rect.height > (2 * rect.width)) { // note: camera is rotated
				enoughPixels = 0;
			}

			// close by obstacles should have a significant size
			// TODO: make use of (configurable) curve instead of fixed values
			if (xClosestBy < 25) {
				if (pixels < 6000) { // verify with falcons/data/internal/vision/multiCam/r2/cam0_20190705_084029.jpg
					enoughPixels = 0;
				} else {
					// printf("INFO   : cam %zu obstacle x closest %d pixels %d\n", camIndex, xClosestBy, pixels);
				}
			} else if (xClosestBy < 50) {
				if (pixels < 4000) {
					enoughPixels = 0;
				} else {
					// printf("INFO   : cam %zu obstacle x closest %d pixels %d\n", camIndex, xClosestBy, pixels);
				}
			} else if (xClosestBy < 100) {
				if (pixels < 3000) {
					enoughPixels = 0;
				}
			} else if (xClosestBy < 150) {
				if (pixels < 2000) {
					enoughPixels = 0;
				}
			} else if (xClosestBy < 200) {
				if (pixels < 1000) {
					enoughPixels = 0;
				}
			}
			// for farther away the configuration slider is used
		}

		// check if there are enough ball pixels to qualify for a ball
		if (enoughPixels) {
			bool valid = false;
			polar.size = pixels;
			polar.xClosestBy = xClosestBy;
			polar.yCenter = yCenter;
			polar.rect = rect;

			// convert from camera dimensions (pixels) to field dimensions (meters)

			// if the ball is on the floor ( z = 0 ) the dewarp (used for line points) can be used to determine the relative location of the ball
			// TODO: make ranges configurable, see dewarpConfig.txt
			float azimuthDewarp = 0.0;
			float elevationDewarp = 0.0;
			float radiusDewarpMm = FLT_MAX;
			if ((xClosestBy >= 0) && (xClosestBy < (ROI_WIDTH / 2)) && (yCenter < ROI_HEIGHT)) {
				// TODO: move the dewarp to he camRecv
				// the correct dewarp already selected on the multiCam level
				dewarp->transform(xClosestBy, yCenter, yField, xField);				// xField and yField result in mm
				if (xField != 0) {
					// dewarp is calibrated for the pixel input at center of the line, while the ball and obstacle detection
					// provide the first dilate pixel, compensate to provide the center of the ball or obstacle
					double xFieldRadiusCorrected = xField + conf->getBall(type).xCenter;						// in mm
					radiusDewarpMm = sqrt(xFieldRadiusCorrected * xFieldRadiusCorrected + yField * yField);		// in mm

					// the dewarp is calibrated for the lines, the xField is closest by
					// TODO: checkout why angle needs to be set negative over here
					// would expect the x and y would have the correct polarity and atan2 should then also be correct
					azimuthDewarp = -atan2(yField, xFieldRadiusCorrected);
					// polar.azimuth = -polar.azimuth - CV_PI/2.0; // new calibration uses inverted angles - 90 degrees
					// Note: the ballDetection is relative to it's own camera, so adding the 90 degrees per camera's will be performed
					// on the higher level
					elevationDewarp = atan2(-CAMERA_HEIGHT, 1e-3 * radiusDewarpMm);	// yikes, careful with MM scaling ...
					valid = true;
				} // if xField
				attemptedDewarp = true;
			} // if xClosestBy

			// if the ball is not on the floor ( z > 0 ) the distance between robot and ball is calculated from the amount of (yellow) ball pixels
			// WARNING: this is not very accurate but should be good enough (the azimuth and elevation should be accurate)
			// the formula is a created from a measured pixel size to distance table
			// this radius is calculated for balls on the floor and in the air
			float radiusPixelsMm = -136.3814 + 24133.4514 / (1 + pow((pixels / 8.401286), 0.5264725));			// in mm

			// the radius determined by the dewarp has a higher accuracy then the radius calculated from the pixels size
			// if the radius from the pixel size is significant smaller (= larger ball) then from the dewarp, assume it is a flying ball
			if (((type == ballType) || (type == ballFarType)) && (radiusPixelsMm < (radiusDewarpMm * 0.30))) { // 20% closer by
				// flying ball
				// the relation between camera pixel and angle is pretty close to linear
				// TODO: perform azimuth and elevation measurement to verify
				float pixelToAngle = conf->getPixelToAngle() / 10000.0;					// camera pixel to radian
				// decreasing y means (looking at the left) means increasing azimuth angle
				float yShifted = ROI_HEIGHT / 2.0 - yCenter;
				float azimuth = pixelToAngle * yShifted;
				polar.azimuth = azimuth;
				// increasing y means (looking up) means increasing elevation angle
				// use the x closest by, so not the center of the object
				float xCentered = xClosestBy - ROI_WIDTH + conf->getXHorizon();
				float elevation = pixelToAngle * xCentered;
				polar.elevation = elevation;
				polar.radius = radiusPixelsMm;
				usedDewarp = false;
				valid = true;
			} else {
				// ball on the floor
				polar.azimuth = azimuthDewarp;
				polar.elevation = elevationDewarp;
				polar.radius = radiusDewarpMm;
				usedDewarp = true;
			} // if radiusPixels

			if (valid) {
				// NOTE: balls outside the dewarp area are estimated based on the camera pixels instead dewarp pixels
#ifdef NONO
				if (camIndex == 3 && type == ballType && printCount == 20) {
					if (usedDewarp ) {
						printf(
								"cam %zu ball pixels %5d xClosestBy %3d yCenter %3d xField %5d yField %5d azimuth %5.1f elevation %5.1f radiusPix %5.0fmm radiusDewarp %5.0fmm dewarp %u\n",
								camIndex, pixels, xClosestBy, yCenter, xField, yField, 180.0 * polar.azimuth / CV_PI,
								180.0 * polar.elevation / CV_PI, radiusPixelsMm, radiusDewarpMm, usedDewarp);
					} else {
						printf(
								"cam %zu ball pixels %5d xClosestBy %3d yCenter %3d xField %5d yField %5d azimuth %5.1f elevation %5.1f radiusPix %5.0fmm radiusDewarp     -   dewarp %u\n",
								camIndex, pixels, xClosestBy, yCenter, xField, yField, 180.0 * polar.azimuth / CV_PI,
								180.0 * polar.elevation / CV_PI, radiusPixelsMm, usedDewarp);

					}
					printCount = 20;
				} else {
					printCount++;
				} // if camIndex
#endif

				char typeStr[32];
				if (type == ballType) {
					sprintf(typeStr, "ball");
				} else if (type == ballFarType) {
					sprintf(typeStr, "ballFar");
				} else if (type == obstacleType) {
					sprintf(typeStr, "obstacle");
				} else if (type == cyanType) {
					sprintf(typeStr, "cyan");
				} else if (type == magentaType) {
					sprintf(typeStr, "magenta");
				} else {
					sprintf(typeStr, "unknown");
				}

// azimuth range: horizontal view angle is around -60 to 60 = 120 degrees (of which 90 is used)
				// however the camera cover of robot 4 generates obstacles with a azimuth of 77 degrees
#define AZIMUTH 90.0
				// elevation range: down -80 degrees to 80 degrees = 160 degree, robot 3 is -81 degrees, add some more headroor
#define ELEVATION 85.0

				if ((polar.azimuth < -(M_PI * AZIMUTH / 180.0)) || (polar.azimuth > (M_PI * AZIMUTH / 180.0))) {
					printf(
							"ERROR     : cam %zu %s azimuth %5.1f out of range, elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else if ((polar.elevation < (-M_PI * ELEVATION / 180.0))
						|| (polar.elevation > ( M_PI * ELEVATION / 180.0))) {
					printf(
							"ERROR     : cam %zu %s elevation %5.1f out of range, azimuth %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.elevation / M_PI, 180.0 * polar.azimuth / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else if ((polar.xClosestBy < 0) || (polar.xClosestBy >= ROI_WIDTH)) {
					printf(
							"ERROR     : cam %zu %s x out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else if ((polar.yCenter < 0) || (polar.yCenter >= ROI_HEIGHT)) {
					printf(
							"ERROR     : cam %zu %s y out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else if ((polar.rect.height < 0) || (polar.rect.height > ROI_HEIGHT)) {
					printf(
							"ERROR     : cam %zu %s rect out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else if ((polar.size < 0) || (polar.size > (ROI_WIDTH * ROI_HEIGHT))) {
					printf(
							"ERROR     : cam %zu %s size out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else if ((polar.radius < 0) || (polar.radius > 20000)) {
					printf(
							"ERROR     : cam %zu %s radius out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
							camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
							polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
				} else {
					// each camera should be able to detect 90 degrees (=> -45 degrees to 45 degrees)
					// because of tolerances a ball on the 45 degree line might be missed
					// with a range of 46 degrees a ball might be reported by 2 camera's
					// TODO: add function that removes double balls (or let world model deal with it)
					float azimuth = conf->getBall(type).azimuth;
					if ((polar.azimuth >= -azimuth) && (polar.azimuth <= azimuth)) {

#ifdef NONO
						// JFEI: I noticed far away obstacles to be accepted, but then projected close by -> ghost obstacles
						// see grab 20180530_190342, in particular the obstacle at cam0
						// so let's reject data outside optiCal range -> we require all data to go via dewarp
						// for obstacles we are even more strict: only within 5 meters
						accept = (xField != 0);
						float CUTOFF_OBSTACLE_METERS = 5.0;
						if ((type == obstacleType) && (polar.radius > CUTOFF_OBSTACLE_METERS * 1e3)) {
							accept = false;
						}
#else
						// try out again how the system behaves for ball's that are far away or flying
						// (which cannot be converted by dewarp)
						// all obstacles are already processed through dewarp because far away and flying
						// obstacles are not generated by the camera (which is proved by a range check in cameraRecieve)
						accept = true;
#endif
						// prevent false positive balls nearby ( < 1 meter) the robot e.g. from reflection by checking if balls are really large nearby
						// polar.radius is in mm
						if (((type == ballType) || (type == ballFarType)) && (polar.radius < 1000)
								&& (pixels < conf->getBall(type).pixelsNearby)) {
							// so it is a ball, within 1 meter but net enough pixels, reject
							accept = false;
						}

						if (accept) {
							positions.push_back(polar);
						}
#ifdef NONO
					} else {
						if (type == ballType) {
							printf("WARNING : cam %zu ball angle of %.0f degrees should be covered by other camera\n",
									camIndex, 180.0 * polar.azimuth / M_PI);
						} else {
							printf("WARNING : cam %zu obstacle angle of %.0f degrees should be covered by other camera\n",
									camIndex, 180.0 * polar.azimuth / M_PI);

						}
#endif
					} // if polar.
				} // valid
			} // if enoughPixels
		} // for contours.size()

#ifdef JFEI
		if (type == ballType) {
			printf("BDET %d %d %d %d %5d %5d %5d %5d %5d %8.3f %8.3f %8.3f\n", accept, enoughPixels, attemptedDewarp,
					usedDewarp, pixels, xClosestBy, yCenter, yField, xField, polar.azimuth, polar.elevation, polar.radius);
			// JFEI:
			// * PTRACE will be gone in rtdb branch
			// * printf is too much to browse through and has no timestamps
			// * -> I'd like this data to be logged into RTDB so we can easily browse & playback .rdl log and diagnose issues
			// *    furthermore, it would enable creation of a basic regression test suite
			// *    (raw vision/pixels in, interpreted ball coordinates out)
			// * this generalizes towards all relevant intermediate computation results and statistics in vision
		}
#else
		// containment for compile warnings about not used variables (the variables are only used in the ifdef above)
		(void) attemptedDewarp;
		(void) usedDewarp;
#endif

	}

	if ((camIndex == 0) && (type == ballType)) {
// determine region of interest (just before the shooter), so this only applies for camera 0
		size_t width = conf->getPossession().width;
		size_t height = conf->getPossession().height;
		size_t x = ROI_HEIGHT / 2 - width / 2;
		size_t y = conf->getPossession().distanceOffset;

// get the region of interest
		inRangeFrame(Rect(y, x, height, width)).copyTo(possessionFrame);

		possessionPixels = countNonZero(possessionFrame);

		if ((((int) possessionPixels) >= conf->getPossession().threshold)) {
			notifyBallPossession(true);
		} else {
			notifyBallPossession(false);
		}
	}

// sort on size
	sort(positions.begin(), positions.end());
	positionsRemoteViewerExportMutex.lock();
	positionsRemoteViewer = positions;
	positionsRemoteViewerExportMutex.unlock();
	exportMutex.unlock();

	if (positions.size() > 0) {
// Notify new ball position
		notifyNewPos();
	}
	busy = false;

#ifdef BALL_LATENCY_MEASUREMENT
	if (type == ballType) {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		double doneTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
		double receiveTime = camRecv->getBallReceiveTime(camIndex);
		printf("INFO    : cam %zu ball calculation time %7.1f ms local %7.1f ms\n", camIndex, 1000.0 * (doneTime - receiveTime), 1000.0 * (doneTime - startTime));
	}
#endif
}

vector<ballSt> ballDetection::getPositions() {
	exportMutex.lock();
	std::vector<ballSt> retVal = positions;
	exportMutex.unlock();
	return retVal;
}

vector<ssize_t> ballDetection::getClosestPixels() {
	exportMutex.lock();
	std::vector<ssize_t> retVal = closestPixels;
	exportMutex.unlock();
	return retVal;
}

vector<ssize_t> ballDetection::getClosestGroups() {
	exportMutex.lock();
	std::vector<ssize_t> retVal = closestGroups;
	exportMutex.unlock();
	return retVal;
}

vector<ballSt> ballDetection::getAndClearPositionsRemoteViewer() {
	positionsRemoteViewerExportMutex.lock();
	std::vector<ballSt> retVal = positionsRemoteViewer;
	positionsRemoteViewer.clear();
	positionsRemoteViewerExportMutex.unlock();
	return retVal;
}

vector<ballSt> ballDetection::getPositionsExport() {
	vector<ballSt> retVal = getPositions();
	for (size_t ii = 0; ii < retVal.size(); ii++) {
		double tmpAzimuth = retVal[ii].azimuth; // TODO: checkout if this is still needed, and if, convert to radians + conf->getExportOffset().rzBall;
// normalize to prevent issues when running of range for remote viewer export
		retVal[ii].azimuth = fmod(2 * CV_PI + tmpAzimuth, 2 * CV_PI);
		retVal[ii].elevation = retVal[ii].elevation; // no fmod, kthxbye
	}
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

cv::Mat ballDetection::getPossessionFrame() {
	exportMutex.lock();
	cv::Mat retVal = possessionFrame.clone();
	exportMutex.unlock();
	return retVal;
}

size_t ballDetection::getAmount() {
	size_t amount = 0;
	exportMutex.lock();
	for (size_t ii = 0; ii < positions.size(); ii++) {
		if (positions[ii].size >= conf->getBall(type).pixels) {
			amount++;
		}
	}
	exportMutex.unlock();
	return amount;
}

void ballDetection::notifyNewPos() {
	vector<ballSt> exportPos = getPositionsExport();

#ifndef NOROS
	if (type != obstacleType) {
		std::vector<ballPositionType> balls;

		for (uint ii = 0; ii < exportPos.size(); ii++) {
			double size = exportPos[ii].size;
			if (size > conf->getBall(type).pixels) {
				// Note: the world model needs to discard balls that are outside the field e.g. yellow something around the field
				// this is depending on the final position, which is determined by the world model instead of the localization on this robot
				double angleRad = exportPos[ii].azimuth;// counter clockwise angle between this robot shooter and the obstacle

				// cam 1 is left of cam 0 = counterclockwise
				angleRad += camIndex * CV_PI / 2.0;			// camIndex * 90 degrees, every camera is 90 degrees rotated

				// from center of this robot to closest edge of ball, there is no need to add a ballRadius to radius\ (ball edge) because in field view the radius is very close to the center
				// exportPos[ii].radius is in mm, while Ros expects meters
				double radius = 0.001f * exportPos[ii].radius;
				double score = size / 100.0f;
				if (score > 1.0) {
					score = 1.0;
				}
				// cout << "ball index " << ii << " size " << size << " score " << score << " << " angle radians " << angleRad << " radius " << radius << endl;

				ballPositionType ball;

				ball.setAngle(angleRad);
				ball.setRadius(radius);
				ball.setConfidence(score);
				ball.setElevation(exportPos[ii].elevation);
				ball.setBallType(type);

				balls.push_back(ball);
			}
		}

		for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); ++iter) {
			if (*iter != NULL) {
				(*iter)->update_own_ball_position(balls, conf->getBallLatencyOffset());
			}
		}
	} else {
		std::vector<obstaclePositionType> obstacles;

		for (uint ii = 0; ii < exportPos.size(); ii++) {
			double size = exportPos[ii].size;
			if (size > conf->getBall(type).pixels) { // type is in this case obstacle
				// Note: the world model needs to discard balls that are outside the field e.g. yellow something around the field
				// this is depending on the final position, which is determined by the world model instead of the localization on this robot
				double angleRad = exportPos[ii].azimuth;// counter clockwise angle between this robot shooter and the obstacle

				// cam 1 is left of cam 0 = counterclockwise
				angleRad += camIndex * CV_PI / 2.0;			// camIndex * 90 degrees, every camera is 90 degrees rotated

				// from center of this robot to closest edge of ball, there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
				// exportPos[ii].radius is in mm, while Ros expects meters
				double radius = 0.001f * exportPos[ii].radius;
				double score = size / 1000.0f;
				if (score > 1.0) {
					score = 1.0;
				}
				// cout << "ball index " << ii << " size " << size << " score " << score << " << " angle radians " << angleRad << " radius " << radius << endl;

				obstaclePositionType obstacle;

				obstacle.setAngle(angleRad);
				obstacle.setRadius(radius);
				obstacle.setConfidence(score);
				int color = 0;					// 0 is black
				obstacle.setColor(color);

				obstacles.push_back(obstacle);
			}
		}

		for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); ++iter) {
			if (*iter != NULL) {
				(*iter)->update_own_obstacle_position(obstacles, conf->getObstacleLatencyOffset());
			}
		}
	}

#endif
}

void ballDetection::notifyBallPossession(bool ballPossession) {
#ifndef NOROS
	for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); iter++) {
		if (*iter != NULL) {
			(*iter)->update_own_ball_possession(ballPossession);
		}
	}
#else
	(void) ballPossession;
#endif
}

void ballDetection::attach(observer *observer) {
	vecObservers.push_back(observer);
}

void ballDetection::detach(observer *observer) {
	vecObservers.erase(std::remove(vecObservers.begin(), vecObservers.end(), observer), vecObservers.end());
}

std::vector<linePointSt> ballDetection::getBallPoints() {
	ballPointListExportMutex.lock();
	std::vector<linePointSt> retVal = ballPointList;
	ballPointListExportMutex.unlock();
	return retVal;
}
