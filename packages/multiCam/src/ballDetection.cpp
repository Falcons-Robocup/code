 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "ballDetection.hpp"

#include <math.h>

using namespace std;
using namespace cv;

ballDetection::ballDetection(cameraReceive *camRecv, configurator *conf, Dewarper *dewarp, BallDistanceEstimator *distanceEstimator, preprocessor *prep,
		size_t type, size_t camIndex) {
	this->camRecv = camRecv;
	this->conf = conf;
	this->dewarp = dewarp;
	this->_distanceEstimator = distanceEstimator;
	this->prep = prep;
	this->type = type;
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
	for (size_t ii = 0; ii < height; ii++) { // create vector for the closest pixels
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


// Search for balls in the related camera (each camera has it's own ball object)
// The balls are provided as "yellow" pixels in the camera image
// It is expected some false ball pixels will be provided
// These will be filtered by the Erode, Dilate and the minimum size
// The dewarp function is used to convert the ball camera pixels to angles
// The angles (azimuth and elevation) are relative to the (mounting-corrected) camera optical axis
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
        printf("ERROR     : ballDetection is not used anymore for obstacleDetection\n");
        exit(EXIT_FAILURE);
    } else {
        printf("ERROR     : ballDetection type %zu unknown, note: ballDetection is not used anymore for obstacleDetection\n", type);
        exit( EXIT_FAILURE);
    }

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


// typically there are points of 1 pixel wide, likely caused by the demosaicing in the FPGA
	erode(inRangeFrame, erodeFrame, conf->getBall(type).erode);			// reduce
// now cluster together what is left
	dilate(erodeFrame, dilateFrame, conf->getBall(type).dilate);			// expand

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

// determine contours that will encapsulate the clusters
	Mat tmp;
	dilateFrame.copyTo(tmp);			// copy dilateFrame otherwise it will be modified by findContours
	findContours(tmp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

	positions.clear();			// start with empty list
	for (size_t ii = 0; ii < contours.size(); ii++) {
		// make a bounding rectangle around each contour to calculate the distance, center and amount of ball pixels
		Rect rect = boundingRect(contours.at(ii));

		// determine the pixel closest by to the ball (radius)
		int xClosestBy = rect.x;

		// determine the center of the ball
		int yCenter = rect.y + rect.height / 2;
		int xCenter = rect.x + rect.width / 2;

		// determine the amount of ball pixels inside the bounding rectangle
		int pixels = countNonZero(inRangeFrame(rect));

		// convert from Cartesian to Polar
		ballSt polar;

		// diagnostics
		bool accept = false;
		bool enoughPixels = pixels > conf->getBall(type).pixels;
		if (enoughPixels && (type == ballFarType)) {
			enoughPixels = pixels < conf->getBall(type).pixelsMax;
		}

		// check if there are enough ball pixels to qualify for a ball
		if (enoughPixels) {

            bool useOldCalculation = true; // experimental compile time toggle, there might be a transformation bug in the calibrated elevation maps (azimuth seems very accurate though)
            bool compareCalculations = false; // extra debugging

			bool valid = false;
			polar.size = pixels;
			polar.xClosestBy = xClosestBy;
			polar.yCenter = yCenter;
			polar.rect = rect;

			// convert from camera dimensions (pixels) to field dimensions (meters)

            // in the early stages of multiCam, only balls on the floor were accepted (z=0) via floor dewarp
            // then flying balls were added but with very inaccurate angles
            // now, front-view dewarp is used
            float azimuthDewarp = 0.0;
            float elevationDewarp = 0.0;
            valid = dewarp->transformFront(xCenter, yCenter, azimuthDewarp, elevationDewarp);
            if (valid)
            {
                polar.azimuth = -azimuthDewarp; // minus sign: robot coordinate system is counter clockwise when seen from top
                polar.elevation = elevationDewarp;
            }
            else
            {
                printf("ERROR     : unexpected failure in dewarp->transformFront for pixels (%d,%d)", xCenter, yCenter);
            }

            if (valid)
            {
                // the distance between robot and ball is calculated from the amount of (yellow) ball pixels
                // this is not very accurate because it is sensitive to lighting ...
                // we use an inline estimation based on dewarp-floor

                // first take the amount of pixels
                // for now, average both dimensions because of possible floor shadow
                float numPixels = 0.5 * (rect.height + rect.width);
                float distance = 0.0;
                bool distanceCalculatedOk = false;

                // feed the estimator, but only when ball is low enough
                if (elevationDewarp < -0.1)
                {
                    int16_t xField = 0;
                    int16_t yField = 0;
                    // calculate distance using dewarp floor lookup tables
                    // take bottom pixel line (xClosestBy) instead of center, for improved precision
                    bool validFloor = dewarp->transformFloor(xClosestBy, yCenter, yField, xField);
                    // xField and yField result in mm
                    if (validFloor)
                    {
                        // dewarp is calibrated for the pixel input at center of the line, while the ball and obstacle detection
                        // provide the first dilate pixel, compensate to provide the center of the ball or obstacle
                        float xFieldRadiusCorrected = xField + conf->getBall(type).xCenter; // in mm
                        float radiusDewarpMm = sqrt(xFieldRadiusCorrected * xFieldRadiusCorrected + yField * yField); // in mm
                        distance = 1e-3 * radiusDewarpMm;
                        distanceCalculatedOk = true;

                        // feed this measurement to the estimator, so it can be used for flying balls
                        _distanceEstimator->feed(numPixels, distance);

                        // workaround: compare with old calculation
                        if (useOldCalculation || compareCalculations)
                        {
                            // old calculation assumed ball to always be on the ground -> use floor lookup tables
                            // this was fine most of the time, but not for bouncing/flying balls
                            // note: radius is floor distance (in mm), which is handy for floorViewer
                            // for both new- and old calculation, the distance is corrected for camera height (75cm) in observerRtDB
                            float azimuthOld = -atan2(yField, xFieldRadiusCorrected);
                            float elevationOld = atan2(-CAMERA_HEIGHT, 1e-3 * radiusDewarpMm);
                            float radiusOld = 1e-3 * radiusDewarpMm;
                            float radiusNew = 1e-3 * radiusDewarpMm; // same calculation, always show [m]

                            if (compareCalculations)
                            {
                                char buf[1024] = {0};
                                static std::map<std::string, bool> seen;
                                sprintf(buf, "dewarp cam=%d dist=%7.3f np=%7.3f xc=%7d xp=%7d yp=%7d xf=%7d yf=%7d xfrc=%7.3f az1=%7.3f az2=%7.3f el1=%7.3f el2=%7.3f r1=%7.3f r2=%7.3f delta_el=%7.3f\n", (int)camIndex, distance, numPixels, xCenter, xClosestBy, yCenter, xField, yField, xFieldRadiusCorrected, polar.azimuth, azimuthOld, polar.elevation, elevationOld, radiusNew, radiusOld, polar.elevation-elevationOld);
                                if (!seen.count(buf))
                                {
                                    printf("%s", buf);
                                    seen[buf] = true;
                                }
                            }
                            if (useOldCalculation)
                            {
                                polar.azimuth = azimuthOld;
                                polar.elevation = elevationOld;
                            }
                        }
                    }
                }

                // fallback in case dewarp floor did not cover it
                if (!distanceCalculatedOk)
                {
                    // query the estimator
                    distance = _distanceEstimator->evaluate(numPixels);
                }

                polar.radius = 1e3 * distance; // for some reason, we calculate in mm for now ... (error-prone!)
                
                if (distance < 0.1) // sanity check
                {
                    // dump data to disk for further analysis (of only latest hit, files are overwritten each time)
                    printf("WARNING   : unexpected ball data, dumping files (numPixels=%.1f r=%.2f az=%.2f el=%.2f)", numPixels, distance, polar.azimuth, polar.elevation);
                    _distanceEstimator->writeSampleData();
                    _distanceEstimator->writeInterpolatedCurve(0, 1000, 1);
                }
            }

			if (valid) {
				char typeStr[32];
				sprintf(typeStr, "ball");

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

						accept = true;

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
						}
#endif
					} // if polar.
				} // valid
			} // if enoughPixels
		} // for contours.size()

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
	std::vector<ballPositionType> balls;

	for (uint ii = 0; ii < exportPos.size(); ii++) {
		double size = exportPos[ii].size;
		if (size > conf->getBall(type).pixels) {
			// Note: the world model needs to discard balls that are outside the field e.g. yellow something around the field
			// this is depending on the final position, which is determined by the world model instead of the localization on this robot
			double angleRad = exportPos[ii].azimuth;// counter clockwise angle between this robot shooter and the ball

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
