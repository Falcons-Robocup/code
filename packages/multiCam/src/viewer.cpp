 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Debug tool to show what is actually going
// Using frames in different stages of the process
// Starting from preProcessing upto lineDetection

#include "viewer.hpp"
#include <iomanip>

using namespace cv;
using namespace std;

viewer::viewer(ballDetection *ballDet[4], ballDetection *ballFarDet[4], cameraReceive *camAnaRecv,
		camSysReceive *camSysRecv, configurator *conf, obstacleDetection *cyanDet, determinePosition *detPos,
		linePointDetection *linePoint, localization *loc, obstacleDetection *magentaDet, obstacleDetection *obstDet[4],
		preprocessor *prep, robotFloor *rFloor, Dewarper *dewarp[4]) {
	this->ballDet[0] = ballDet[0];
	this->ballDet[1] = ballDet[1];
	this->ballDet[2] = ballDet[2];
	this->ballDet[3] = ballDet[3];
	this->ballFarDet[0] = ballFarDet[0];
	this->ballFarDet[1] = ballFarDet[1];
	this->ballFarDet[2] = ballFarDet[2];
	this->ballFarDet[3] = ballFarDet[3];
	this->camAnaRecv = camAnaRecv;
	this->camSysRecv = camSysRecv;
	this->conf = conf;
	this->cyanDet = cyanDet;
	this->detPos = detPos;
	this->linePoint = linePoint;
	this->loc = loc;
	this->magentaDet = magentaDet;
	this->obstDet[0] = obstDet[0];
	this->obstDet[1] = obstDet[1];
	this->obstDet[2] = obstDet[2];
	this->obstDet[3] = obstDet[3];
	this->prep = prep;
	this->rFloor = rFloor;
	this->dewarp[0] = dewarp[0];
	this->dewarp[1] = dewarp[1];
	this->dewarp[2] = dewarp[2];
	this->dewarp[3] = dewarp[3];

	exportMutex.lock();
	key = 0;
	textOverlay = true; // default we want the essential information available in the viewers, but it might inconvenient when doing a calibration
	pause = true;
	viewMode = bgr;
	viewModeText = ""; // will be set in show function
	ballView = false; // will be set in show function
	cyanView = false;
	magentaView = false;
	floorOverlay = false;
    dewarpOverlay = false;
	showCostField = false;
	help = false;
	selectWindow = "camera view, press h for help";
	floorWindow = "floor view";
	whiteBallWindow = "white ball windows";
	busy = false;
	cyans.clear();
	magentas.clear();
	locations.clear();

	camHeight = conf->getCameraHeight();
	camWidth = conf->getCameraWidth();
	floorHeight = rFloor->getHeight();
	floorWidth = rFloor->getWidth();

	// create once the color image that is later used to copy the line points in the round viewer.
	linePointsColorFrame = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC3);
	linePointsColorFrame = Scalar(255, 0, 0);

	// create once the color image that is later used to copy all line points in the round viewer.
	linePointsAllColorFrame = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC3);
	linePointsAllColorFrame = Scalar(100, 100, 100);

	// create once the color image that is later used to copy the rejected line points in the round viewer.
	linePointsRejectColorFrame = Mat::zeros(conf->getCameraViewerHeight(), conf->getCameraViewerWidth(), CV_8UC3);
	linePointsRejectColorFrame = Scalar(0, 0, 255);

	selectFrame = Mat::zeros((2 * camWidth + camHeight) / 2, (2 * camWidth + camHeight) / 2, CV_8UC3); // needs to hold all 4 camera's but scaled to 50%
	selectFrameExport = Mat::zeros((2 * camWidth + camHeight) / 2, (2 * camWidth + camHeight) / 2, CV_8UC3);

	// create once the color image that is later used to copy the floor lines in the rectangular (floor) viewer
	floorColorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);
	floorColorFrame = COLOR_LINE;

	floorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);
	floorFrameExport = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);

	whiteBallFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC1);
	whiteBallFrameExport = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC1);

	// create a frame for each camera view
	for (size_t cam = 0; cam < 4; cam++) {
		camFrameList[cam] = Mat::zeros(camHeight / 2, camWidth / 2, CV_8UC3);
	}

	period = 0.4;
	nextTick = 0;
	nextLogFileTick = 0;

	exportMutex.unlock();
}

// show both viewers and manage short keys
void viewer::update() {
	if (getTickCount() > nextTick) {
		nextTick = getTickCount() + (int64) (period * getTickFrequency());

		// acquire the robot locations (with best probability), ball locations and obstacles.
		// do this only once to keep consistency between all users of this information (e.g. drawing in viewers, logging, ..
		cyans = cyanDet->getPositions();
		locations = detPos->getLocList();
		magentas = magentaDet->getPositions();

		roundViewerUpdate();
		floorUpdate();

		exportMutex.lock();
		selectFrameExport = selectFrame.clone();
		floorFrameExport = floorFrame.clone();
		whiteBallFrameExport = whiteBallFrame.clone();
		exportMutex.unlock();

		if (conf->getGuiEnabled()) {
			imshow(floorWindow, floorFrameExport);
			imshow(selectWindow, selectFrameExport);
// #define SHOW_WHITE_BALL_PIXELS
#ifdef SHOW_WHITE_BALL_PIXELS
			imshow(whiteBallWindow, whiteBallFrameExport);
#endif
		}
	}

    // clicking for coordinates (via dewarp)
    cv::setMouseCallback(selectWindow, onMouse, this);

	key = waitKey(20);
	switch (key) {
	case 'b':
		viewMode = bgr;
		break;
	case 'c':
		conf->setCalibrateMode(!conf->getCalibrateMode());
		if (conf->getCalibrateMode()) {
			conf->setManualMode(true); // manual mode required for calibration
		} else {
			conf->setManualMode(false);
		}
		break;
	case 'h':
		help = !help;
		break;
    case 'i': // i from isolines
        dewarpOverlay = !dewarpOverlay;
        break;
	case 'k':
		floorOverlay = !floorOverlay;
		break;
	case 'g':
		showCostField = !showCostField;
		break;
	case ' ': // space
		pause = !pause;
		break;
	case 'p':
		viewMode--;
		if (viewMode < 0) {
			viewMode = last - 1;
		}
		break;
	case 'm':
		conf->setManualMode(!conf->getManualMode());
		break;
	case 'n':
		viewMode++;
		if (viewMode >= last) {
			viewMode = 0;
		}
		break;
	case 'o':
		textOverlay = !textOverlay;
		break;
	case 'u':
		// move from continues viewing to every x viewing for e.g. slow remote WiFi connection
		if (period > 0.1) {
			period = 0.03;
		} else {
			period = 0.4;
		}
		break;
	case 'w':
		cout << "write config (not implemented)" << endl;
		break;
	default:
		break;
	}
	printStats();
	busy = false;
}

// update data for round (distorted) viewer
void viewer::roundViewerUpdate() {
	ballView = false;
	cyanView = false;
	magentaView = false;
	Mat camFrame;
	vector<Point> cartesian;
	vector<linePointSt> lineCartesian;

	selectFrame = Mat::zeros((2 * camWidth + camHeight) / 2, (2 * camWidth + camHeight) / 2, CV_8UC3); // needs to hold all 4 camera's but scaled to 50%

	switch (viewMode) {
	case bgr:
		for (size_t cam = 0; cam < 4; cam++) {
			// camAnaRecv->getCameraFrame(cam).copyTo(camFrame);
			camSysRecv->getCameraFrame(cam).copyTo(camFrame);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "BGR";
		break;
	case ball:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(ballDet[cam]->getInRangeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Ball";
		ballView = true;
		break;
	case ballErode:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(ballDet[cam]->getErodeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Ball erode";
		ballView = true;
		break;
	case ballDilate:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(ballDet[cam]->getDilateFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Ball dilate";
		ballView = true;
		break;
	case ballFar:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(ballFarDet[cam]->getInRangeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Ball far";
		ballView = true;
		break;
	case ballFarErode:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(ballFarDet[cam]->getErodeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Ball far erode";
		ballView = true;
		break;
	case ballFarDilate:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(ballFarDet[cam]->getDilateFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Ball far dilate";
		ballView = true;
		break;
	case cyan:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(cyanDet->getInRangeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Cyan";
		cyanView = true;
		break;
	case magenta:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(magentaDet->getInRangeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Magenta";
		magentaView = true;
		break;
	case obstacleSel:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(obstDet[cam]->getInRangeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Obstacle";
		break;
	case obstacleErode:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(obstDet[cam]->getErodeFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Obstacle erode";
		break;
	case obstacleDilate:
		for (size_t cam = 0; cam < 4; cam++) {
			cvtColor(obstDet[cam]->getDilateFrame(), camFrame, COLOR_GRAY2BGR);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "Obstacle dilate";
		break;
	default:
		for (size_t cam = 0; cam < 4; cam++) {
			// camAnaRecv->getCameraFrame(cam).copyTo(camFrame);
			camSysRecv->getCameraFrame(cam).copyTo(camFrame);
			resize(camFrame, camFrame, Size(), 0.5, 0.5, INTER_LINEAR); // the viewer is 50% of real size
			camFrame.copyTo(camFrameList[cam]);
		}
		viewModeText = "no selection";
		break;
	}

	if (!conf->getCalibrateMode()) {
		// do not display balls and obstacles in calibration mode

		drawObstacles();

		if ((viewMode != ball) && (viewMode != ballErode) && (viewMode != ballDilate) && (viewMode != obstacleSel)
				&& (viewMode != obstacleErode) && (viewMode != obstacleDilate)) {
			for (size_t cam = 0; cam < 4; cam++) {
				if (floorOverlay) {
					// ## floor ##
					Scalar floorColor = { 0, 255, 0 };
					lineCartesian = camAnaRecv->getFloorPoints(cam);
					for (size_t ii = 0; ii < lineCartesian.size(); ii++) {
						int xBegin = lineCartesian[ii].xBegin / 2; // camera viewer is 50% of camera actual camera size
						int xEnd = lineCartesian[ii].xEnd / 2;
						int yBegin = lineCartesian[ii].yBegin / 2;
						int yEnd = lineCartesian[ii].yEnd / 2;
						line(camFrameList[cam], Point(xBegin, yBegin), Point(xEnd, yEnd), floorColor, 1, 8);
					}
				}

                // contour lines (isolines) from dewarp, first, to draw below any other objects
                if (dewarpOverlay) {
                    // colors are BGR
                    // first the horizon: elevation == 0, color purple
                    drawIsolineElevation(cam,      0.0, Vec3b(168,  50, 123));
                    // center view axis: azimuth == 0, color cyan
                    drawIsolineAzimuth  (cam,      0.0, Vec3b(168, 160,  50));
                    // camera nominal view range of 90 degrees, pixels outside are bonus (overlap with other cam)
                    drawIsolineAzimuth  (cam, -CV_PI/4, Vec3b(168, 160,  50));
                    drawIsolineAzimuth  (cam,  CV_PI/4, Vec3b(168, 160,  50));
                    drawIsolineAzimuth  (cam, -CV_PI/8, Vec3b(168, 160,  50), 6);
                    drawIsolineAzimuth  (cam,  CV_PI/8, Vec3b(168, 160,  50), 6);
                    // 2m and 6m distance
                    drawIsolineElevation(cam, -atan2(CAMERA_HEIGHT, 2.0), Vec3b(168,  50, 123));
                    drawIsolineElevation(cam, -atan2(CAMERA_HEIGHT, 6.0), Vec3b(168,  50, 123));
                }
				
				// ## lines ##
				Scalar lineColor = { 255, 0, 0 };
				// in case the line points from a board are not received, the information can only be used once
				// show the information in the viewer used by the localization, instead of the information from the receiver (which might be old)
				lineCartesian = linePoint->getLinePoints(cam);
				for (size_t ii = 0; ii < lineCartesian.size(); ii++) {
					int xBegin = lineCartesian[ii].xBegin / 2; // camera viewer is 50% of camera actual camera size
					int xDistanceOffset = conf->getLine().distanceOffset / 2;
					if (xBegin < xDistanceOffset) {
						xBegin = xDistanceOffset;
					}
					int xEnd = lineCartesian[ii].xEnd / 2;
					int xDistance = conf->getLine().distance / 2;
					if (xEnd >= xDistance) {
						xEnd = xDistance;
					}
					int yBegin = lineCartesian[ii].yBegin / 2;
					int yEnd = lineCartesian[ii].yEnd / 2;
					if (xEnd >= xBegin) {
						// xEnd might be lower then xBegin because of the distance offset
						line(camFrameList[cam], Point(xBegin, yEnd), Point(xEnd, yBegin), lineColor, 1, 8);
					}
				}

#ifdef NONO
				// ## balls ##
				Scalar ballColor = {0, 255, 255};
				lineCartesian = ballDet[cam]->getBallPoints();
				for (size_t ii = 0; ii < lineCartesian.size(); ii++) {
					int xBegin = lineCartesian[ii].xBegin / 2; // camera viewer is 50% of camera actual camera size
					int xEnd = lineCartesian[ii].xEnd / 2;
					int yBegin = lineCartesian[ii].yBegin / 2;
					int yEnd = lineCartesian[ii].yEnd / 2;
					line(camFrameList[cam], Point(xBegin, yBegin), Point(xEnd, yEnd), ballColor, 1, 8);
				}

				// ## obstacles ##
				Scalar obstacleColor = {255, 0, 255};
				lineCartesian = obstDet[cam]->getBallPoints();
				for (size_t ii = 0; ii < lineCartesian.size(); ii++) {
					int xBegin = lineCartesian[ii].xBegin / 2; // camera viewer is 50% of camera actual camera size
					int xEnd = lineCartesian[ii].xEnd / 2;
					int yBegin = lineCartesian[ii].yBegin / 2;
					int yEnd = lineCartesian[ii].yEnd / 2;
					line(camFrameList[cam], Point(xBegin, yBegin), Point(xEnd, yEnd), obstacleColor, 1, 8);
				}
#endif
				// ## ball contours ##
				vector<ballSt> ballList = ballDet[cam]->getPositionsExport();
				for (size_t ii = 0; ii < ballList.size(); ii++) {
					Rect rectScaled;
					rectScaled.x = ballList[ii].rect.x / 2;
					rectScaled.y = ballList[ii].rect.y / 2;
					rectScaled.width = ballList[ii].rect.width / 2;
					rectScaled.height = ballList[ii].rect.height / 2;
					rectangle(camFrameList[cam], rectScaled, COLOR_BALL, 1);
				}

				// ## ballFar contours ##
				vector<ballSt> ballFarList = ballFarDet[cam]->getPositionsExport();
				for (size_t ii = 0; ii < ballFarList.size(); ii++) {
					Rect rectScaled;
					rectScaled.x = ballFarList[ii].rect.x / 2;
					rectScaled.y = ballFarList[ii].rect.y / 2;
					rectScaled.width = ballFarList[ii].rect.width / 2;
					rectScaled.height = ballFarList[ii].rect.height / 2;
					rectangle(camFrameList[cam], rectScaled, COLOR_BALL_FAR, 1);
				}

				// ## ball possession (in front of the shooter, which is camera 0) ##
				if (cam == 0) {
					// only draw the ROI rectangle in the camera viewer, nothing functional!
					size_t width = conf->getPossession().width;
					size_t height = conf->getPossession().height;
					size_t x = ROI_HEIGHT / 2 - width / 2;
					size_t y = conf->getPossession().distanceOffset;

					Point leftTop = Point(y / 2, x / 2); // resize to 50%
					Point rightBottom = Point((y + height) / 2, (x + width) / 2); // resize to 50%

					Scalar color(255, 255, 255); // white (to distinguish from ball detection)
					rectangle(camFrameList[cam], leftTop, rightBottom, color, 1);
				}

				// ## show nearby obstacle pixels ##
				vector<ssize_t> closestPixels = obstDet[cam]->getClosestPixels();
				vector<ssize_t> closestGroups = obstDet[cam]->getClosestGroups();
				for (size_t yy = 0; yy < closestPixels.size(); yy++) {
					ssize_t xx = closestPixels[yy];
					if (xx != -1) {
						// use 4 different colors to see the transition from one obstacle to the next
						Scalar color = Scalar(0, 0, 0); // black
						if ((closestGroups[yy] % 4) == 0) {
							color = Scalar(0, 0, 255); // red
						} else if ((closestGroups[yy] % 4) == 1) {
							color = Scalar(255, 0, 0); // blue
						} else if ((closestGroups[yy] % 4) == 2) {
							color = Scalar(255, 0, 255); // purple
						} else {
							color = Scalar(255, 255, 0); // cyan
						}
						line(camFrameList[cam], Point(xx / 2, yy / 2), Point(xx / 2, yy / 2), color, 2);
					}
				}

				// ## show camera cover mask lines (which are used to block the obstacles and balls)
				Scalar maskColor(255, 255, 255);
				// scale for half the image size
				// triangle
				Point leftFarAway(conf->getMask().leftFarAway / 2, 0);
				Point leftCloseby(0, conf->getMask().leftCloseby / 2);
				Point rightFarAway(conf->getMask().rightFarAway / 2, (ROI_HEIGHT - 1) / 2);
				Point rightCloseby(0, conf->getMask().rightCloseby / 2);
				line(camFrameList[cam], leftFarAway, leftCloseby, maskColor, 1);
				line(camFrameList[cam], rightFarAway, rightCloseby, maskColor, 1);
				// printf("INFO     : leftFarAway %d leftCloseby %d rightCloseby %d rightFarAway %d\n",
				//		conf->getMask().leftFarAway, conf->getMask().leftCloseby,
				//		conf->getMask().rightCloseby, conf->getMask().rightFarAway);
				// block line
				Point blockLeft = Point(conf->getBall(obstacleType).distanceOffset / 2, 0);
				Point blockRight = Point(conf->getBall(obstacleType).distanceOffset / 2, (ROI_HEIGHT - 1) / 2);
				line(camFrameList[cam], blockLeft, blockRight, maskColor, 1);

				// ## obstacle contours ##
				vector<obstacleSt> obstacleList = obstDet[cam]->getPositionsExport();
				for (size_t ii = 0; ii < obstacleList.size(); ii++) {
					Rect rectScaled;
					rectScaled.x = obstacleList[ii].rect.x / 2;
					rectScaled.y = obstacleList[ii].rect.y / 2;
					rectScaled.width = obstacleList[ii].rect.width / 2;
					rectScaled.height = obstacleList[ii].rect.height / 2;
					rectangle(camFrameList[cam], rectScaled, COLOR_MAGENTA, 1);
				}
			}
		}
	}

	// show front camera in camera viewer = move and rotate 90 degrees
	transpose(camFrameList[0], camFrameList[0]);
	flip(camFrameList[0], camFrameList[0], 0);
	camFrameList[0].copyTo(selectFrame.colRange(camWidth / 2, camWidth / 2 + camHeight / 2).rowRange(0, camWidth / 2));

	// show left camera in camera viewer = move and rotate 180 degrees
	flip(camFrameList[1], camFrameList[1], -1);
	camFrameList[1].copyTo(selectFrame.colRange(0, camWidth / 2).rowRange(camWidth / 2, camWidth / 2 + camHeight / 2));

	// show rear camera in camera viewer = move and rotate 270 degrees
	transpose(camFrameList[2], camFrameList[2]);
	flip(camFrameList[2], camFrameList[2], 1);
	camFrameList[2].copyTo(
			selectFrame.colRange(camWidth / 2, camWidth / 2 + camHeight / 2).rowRange(camWidth / 2 + camHeight / 2,
					2 * camWidth / 2 + camHeight / 2));

	// show right camera in camera viewer = move
	camFrameList[3].copyTo(
			selectFrame.colRange(camWidth / 2 + camHeight / 2, 2 * camWidth / 2 + camHeight / 2).rowRange(camWidth / 2,
					camWidth / 2 + camHeight / 2));

	if (textOverlay) {
		printText(); // makes image darker when in help mode to make reading easier
	}
}

// update data for rectangular robot floor viewer
void viewer::floorUpdate() {
	floorFrame = Mat::zeros(floorHeight, floorWidth, CV_8UC3);
	floorFrame = COLOR_GRASS;

	if (showCostField) {
		vector<Mat> floorTmp;
		floorTmp.push_back(Mat::zeros(floorHeight, floorWidth, CV_8UC1));
		floorTmp.push_back(rFloor->getCostField());
		floorTmp.push_back(Mat::zeros(floorHeight, floorWidth, CV_8UC1));
		cv::merge(floorTmp, floorFrame);
	} else {
		floorColorFrame.copyTo(floorFrame, (rFloor->getRefField() != 0)); // virtual field used for human interpretation
	}

	if (!conf->getCalibrateMode()) {
		// do not display balls, obstacles and search information in calibration mode

		if (locations.size() > 0) {
			// so we have a lock, get current position
			positionStDbl robot = locations[0].pos; // use the "best" found location as the projected robot position
			drawFloorObstacles(robot);
			drawFloorCyans(robot);
			drawFloorMagentas(robot);
			drawFloorBalls(robot);
			drawFloorBallsFar(robot);
			for (unsigned int ii = 0; ii < locations.size(); ii++) {
				if (ii == 0) {
					if (locations[ii].lastActive < 5) {
						drawFloorLinePoints(locations[ii].pos, COLOR_LINEPOINTS);
						drawFloorRobot(locations[ii].pos, COLOR_ROBOT);
					} else {
						drawFloorLinePoints(locations[ii].pos, COLOR_LINEPOINTS_DARK);
						drawFloorRobot(locations[ii].pos, COLOR_ROBOT_DARK);
					}

				}
			}
		}
	}
	if (textOverlay) {
		floorPrintText();
	}
}

// print the text in the rectangular robot floor viewer
void viewer::floorPrintText() {
	char buf[256];
	int line = 20;
	Scalar color(0, 255, 255); // yellow
	std::stringstream strForm;
	strForm << "Falcon " << conf->getRobot();
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	strForm << "Frame " << setw(6) << camAnaRecv->getAnaFrameCounter(0); // show frame counter of camera 0 (to which the other camera's will be synchronized)
	strForm << " uptime " << setw(6) << fixed << std::setprecision(1) << prep->getUptime();
	strForm << " fps " << setw(4) << fixed << std::setprecision(1) << prep->getFps() << " " << loc->getFps();
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");
	strForm << "Cam delta " << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(1) << " ";
	strForm << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(2) << " ";
	strForm << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(3) << " ms";
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	// ball possession is determined by looking at ball pixels before the shooter, which is on camera 0
	sprintf(buf, "Balls %zu+%zu+%zu+%zu far %zu+%zu+%zu+%zu possession %3zu", ballDet[0]->getAmount(),
			ballDet[1]->getAmount(), ballDet[2]->getAmount(), ballDet[3]->getAmount(), ballFarDet[0]->getAmount(),
			ballFarDet[1]->getAmount(), ballFarDet[2]->getAmount(), ballFarDet[3]->getAmount(),
			ballDet[0]->getPossessionPixels());
	putText(floorFrame, buf, Point(10, line), 1, 1, color, 1);
	line += 20;

	sprintf(buf, "Obstacles %zu+%zu+%zu+%zu", obstDet[0]->getAmount(), obstDet[1]->getAmount(), obstDet[2]->getAmount(),
			obstDet[3]->getAmount());
	putText(floorFrame, buf, Point(10, line), 1, 1, color, 1);
	line += 20;

	// print localization results
	strForm << "Line Points " << setw(3) << linePoint->getLinePointsPolar().size();
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 40;
	strForm.str("");
	strForm << "Score ";
	for (size_t ii = 0; ii < locations.size(); ii++) {
		strForm << setw(5) << fixed << std::setprecision(3) << locations[ii].score << " ";
	}
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	strForm << "Age   ";
	for (size_t ii = 0; ii < locations.size(); ii++) {
		strForm << setw(5) << locations[ii].age << " ";
	}
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	strForm << "Active ";
	for (size_t ii = 0; ii < locations.size(); ii++) {
		strForm << setw(5) << locations[ii].lastActive << " ";
	}
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	strForm << "Tries  ";
	for (size_t ii = 0; ii < locations.size(); ii++) {
		strForm << setw(5) << locations[ii].numberOfTries << " ";
	}
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	strForm << "Pixels ";
	for (size_t ii = 0; ii < locations.size(); ii++) {
		strForm << setw(5) << locations[ii].amountOnFloor << " ";
	}
	putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 40;
	strForm.str("");

	if (locations.size() > 0) {
		putText(floorFrame, "Position    x       y        rz", Point(10, line), 1, 1, color, 1);
		line += 20;
		strForm.str("");
		strForm << "multiCam  " << std::setprecision(1) << setw(6) << locations[0].pos.x << " " << setw(6)
				<< locations[0].pos.y << " " << setw(6) << locations[0].pos.rz;
		putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
		line += 20;
		strForm.str("");

		detPosSt locGood = detPos->getGoodEnoughLocExport();
		if (locGood.goodEnough) {
			strForm << "Good    " << std::setprecision(1) << setw(6) << locGood.pos.x << " " << setw(6) << locGood.pos.y
					<< " " << setw(6) << locGood.pos.rz;
			putText(floorFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
			line += 20;
			strForm.str("");
		}

		detPosSt locRos = detPos->getGoodEnoughLocExportRos();
		if (locRos.goodEnough) {
			double angleDeg = locRos.pos.rz * 180.0 / CV_PI;
			if (angleDeg > 180.0) {
				angleDeg -= 360;
			}
			sprintf(buf, "Ros  %4.1f cm y %4.1f cm %4.1f deg", 100.0 * locRos.pos.y, -100.0 * locRos.pos.x, angleDeg);
			putText(floorFrame, buf, Point(10, line), 1, 1, color, 1);
			line += 20;
		}
	}

	if (conf->getManualMode()) {
		putText(floorFrame, "Manual mode", Point(10, line), 1, 1, color, 1);
		line += 20;
	}
	if (conf->getCalibrateMode()) {
		putText(floorFrame, "Calibration mode", Point(10, line), 1, 1, color, 1);
		line += 20;
	}
}

void viewer::printText() {
	char buf[256];
	int line = 20;
	Scalar color(0, 255, 255); // yellow
	Scalar color_err(0, 0, 255); // red
	std::stringstream strForm;
	if (help) {
		selectFrame = selectFrame * 0.5; // make background darker to make help text easier to read
	}
	strForm << "Falcon " << conf->getRobot();
	putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	strForm << "Frame " << setw(6) << camAnaRecv->getAnaFrameCounter(0); // show frame counter of camera 0 (to which the other camera's will be synchronized)
	strForm << " uptime " << setw(6) << fixed << std::setprecision(1) << prep->getUptime();
	strForm << " fps " << setw(4) << fixed << std::setprecision(1) << prep->getFps() << " " << loc->getFps();
	putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");
	strForm << "Cam delta " << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(1) << " ";
	strForm << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(2) << " ";
	strForm << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(3) << " ms";
	putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");
	putText(selectFrame, viewModeText, Point(10, line), 1, 1, color, 1);
	line += 20;

	strForm << "Line Points " << setw(3) << linePoint->getLinePointsPolar().size();
	putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
	line += 20;
	strForm.str("");

	if (ballView || viewMode == bgr) {
		line += 20;
		bool ballFound = false;
		size_t ballSize = 0;
		double ballRadius = 0.0;
		double ballAngle = 0.0;
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> balls = ballDet[cam]->getPositions();
			for (size_t ii = 0; ii < balls.size(); ii++) {
				if (balls[ii].size >= conf->getBall(ballType).pixels) {
					ballFound = true;
					if (((double) balls[ii].size) > ballSize) {
						ballSize = balls[ii].size;
						ballAngle = balls[ii].azimuth;
						// cam 1 is left of cam 0 = counterclockwise
						ballAngle += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated
						ballRadius = balls[ii].radius; // radius in mm
					}
				}
			}
		}

		if (ballFound) {
			strForm << "Ball size " << setw(6) << ballSize;
			strForm << " angle " << setw(6) << std::setprecision(1) << 180.0 * ballAngle / CV_PI;
			strForm << " deg radius " << setw(6) << ballRadius / 10.0 << " cm";
			putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
		}
		line += 20;
		strForm.str("");
	}
	if (ballFarView || viewMode == bgr) {
		bool ballFound = false;
		size_t ballSize = 0;
		double ballRadius = 0.0;
		double ballAngle = 0.0;
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> balls = ballFarDet[cam]->getPositions();
			for (size_t ii = 0; ii < balls.size(); ii++) {
				if (balls[ii].size >= conf->getBall(ballFarType).pixels) {
					ballFound = true;
					if (((double) balls[ii].size) > ballSize) {
						ballSize = balls[ii].size;
						ballAngle = balls[ii].azimuth;
						// cam 1 is left of cam 0 = counterclockwise
						ballAngle += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated
						ballRadius = balls[ii].radius; // radius in mm
					}
				}
			}
		}

		if (ballFound) {
			strForm << "Ball far size " << setw(6) << ballSize;
			strForm << " angle " << setw(6) << std::setprecision(1) << 180.0 * ballAngle / CV_PI;
			strForm << " deg radius " << setw(6) << ballRadius / 10.0 << " cm";
			putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
			line += 20;
			strForm.str("");
			line += 20;
			strForm.str("");
		} else {
			line += 20;
		}
	}
	if (cyanView || viewMode == bgr) {
		line += 20;
		if (cyans.size() > 0) {
			putText(selectFrame, "Cyan size azimuth elevation radius radRos", Point(10, line), 1, 1, color, 1);
			line += 20;
			strForm.str("");
		} else {
			line += 20;
		}
		for (size_t ii = 0; ii < cyans.size(); ii++) {
			if (cyans[ii].size >= conf->getBall(cyanType).pixels) {
				if (ii < 3) {
					strForm << setw(6) << cyans[ii].size << " ";
					strForm << setw(6) << std::setprecision(1) << cyans[ii].azimuth << " ";
					strForm << setw(6) << std::setprecision(1) << cyans[ii].elevation << " ";
					strForm << setw(6) << cyans[ii].radius << " ";
					strForm << setw(6) << setprecision(2) << (0.02f * cyans[ii].radius);
					putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
					line += 20;
					strForm.str("");
				}
			}
		}
	}
	if (magentaView || viewMode == bgr) {
		line += 20;
		if (magentas.size() > 0) {
			putText(selectFrame, "Magenta size azimuth elevation radius radRos", Point(10, line), 1, 1, color, 1);
			line += 20;
			strForm.str("");
		} else {
			line += 20;
		}
		for (size_t ii = 0; ii < magentas.size(); ii++) {
			if (magentas[ii].size >= conf->getBall(magentaType).pixels) {
				if (ii < 3) {
					strForm << setw(6) << magentas[ii].size << " ";
					strForm << setw(6) << std::setprecision(1) << magentas[ii].azimuth << " ";
					strForm << setw(6) << std::setprecision(1) << magentas[ii].elevation << " ";
					strForm << setw(6) << magentas[ii].radius << " ";
					strForm << setw(6) << setprecision(2) << (0.02f * magentas[ii].radius);
					putText(selectFrame, strForm.str(), Point(10, line), 1, 1, color, 1);
					line += 20;
					strForm.str("");
				}
			}
		}
	}

	// provide raspi camera statistics in camera viewer
	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		camSystemSt cs = camSysRecv->getCamSystem(camIndex);

		// if (cs.updated) { // TODO: use cs.update to show if the data becomes old

		uint16_t sysApplUptime = cs.sysApplUptime;
		uint16_t cpuUptime = cs.cpuUptime;
		uint16_t anaApplUptime = camAnaRecv->getAnaApplUptime(camIndex);

		// convert to hours minutes and seconds
		hhMmSsSt sysUp = camSysRecv->secondsToHhMmSs(sysApplUptime);
		hhMmSsSt cpuUp = camSysRecv->secondsToHhMmSs(cpuUptime);
		hhMmSsSt anaUp = camSysRecv->secondsToHhMmSs(anaApplUptime);

		// set position in viewer where to print the information for each camera
		// so it becomes obvious to which image it relates
		int line, pixel;
		if (camIndex == 0) {
			// top in viewer
			line = 30;
			pixel = 710;
		} else if (camIndex == 1) {
			// left in viewer
			line = 190;
			pixel = 10;
		} else if (camIndex == 2) {
			// bottom in viewer
			line = 800;
			pixel = 100;
		} else {
			// right in viewer
			line = 720;
			pixel = 810;
		}

		sprintf(buf, "cam %zu", camIndex);
		putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		line += 20;

		float camValAvg = (float) camAnaRecv->getCamValAverage(camIndex);
		sprintf(buf, "cam val average %.0f", camValAvg);
		float camValAvgOther = 0;
		for (size_t ii = 0; ii < 4; ii++) {
			if (ii != camIndex) {
				camValAvgOther += (float) camAnaRecv->getCamValAverage(ii);
			}
		}

		camValAvgOther = camValAvgOther / 3.0;
		// within 30% lower or 50% higher then average over other 3 cameras
		if ((camValAvg < (camValAvgOther * 0.7)) || (camValAvg > (camValAvgOther * 1.5))) {
			// the camera value for this camera differers to much related to the other cameras
			putText(selectFrame, buf, Point(pixel, line), 1, 1, { 0, 0, 255 }, 1); // red
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		}
		line += 20;

		sprintf(buf, "balls %2zu far %2zu obstacles %zu", ballDet[camIndex]->getAmount(),
				ballFarDet[camIndex]->getAmount(), obstDet[camIndex]->getAmount());
		putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		line += 20;

		if (camIndex == 0) {
			// ball possession is determined by looking at ball pixels before the shooter, which is on camera 0
			sprintf(buf, "ball possession %zu", ballDet[camIndex]->getPossessionPixels());
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
			line += 20;
		}

		sprintf(buf, "line points lAxis %3zu sAxis %3zu", camAnaRecv->getLinePointsLongAxisAmount(camIndex),
				camAnaRecv->getLinePointsShortAxisAmount(camIndex));
		putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		line += 20;

		sprintf(buf, "cpu temp %2u gpu %2u cmos %2d", cs.cpuTemp, cs.gpuTemp, cs.cmosTemp);
		if (cs.cpuTemp >= 70 || cs.gpuTemp >= 70 || cs.cmosTemp > 55) {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color_err, 1);
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);

		}
		line += 20;

		float cpuLoad = cs.cpuLoad / 32.0;
		sprintf(buf, "cpu load %4.2f", cpuLoad);
		if (cpuLoad < 2.0 || cpuLoad > 3.5) {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color_err, 1);
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		}
		line += 20;

		sprintf(buf, "md5sum %08x", cs.md5sumPart0);
		putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		line += 20;

		// make clear a reboot has occurred on the raspi
		struct timeval tv;
		gettimeofday(&tv, NULL);
		double localTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
		double deltaTime = localTime - cs.rebootTime;
		uint32_t rebootTime = 0;
		if (deltaTime < 60) {
			rebootTime = (uint32_t) deltaTime;
		}
		sprintf(buf, "volt %u frq %u thrt %u soft %u boot %u", cs.underVoltage, cs.frequencyCapped, cs.throttling,
				cs.softTempLimit, rebootTime);
		if (cs.underVoltageOccured || cs.frequencyCappedOccured || cs.throttlingOccured || cs.softTempLimitOccured
				|| (rebootTime > 0)) {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color_err, 1);
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);

		}
		line += 20;

		sprintf(buf, "raspi    uptime %02u:%02u:%02u", cpuUp.hours, cpuUp.minutes, cpuUp.seconds);
		if (cpuUptime < 60) {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color_err, 1);
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);
		}
		line += 20;

		sprintf(buf, "system  uptime %02u:%02u:%02u", sysUp.hours, sysUp.minutes, sysUp.seconds);
		if (sysApplUptime < 10) {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color_err, 1);
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);

		}
		line += 20;

		sprintf(buf, "analyzer uptime %02u:%02u:%02u", anaUp.hours, anaUp.minutes, anaUp.seconds);
		if (anaApplUptime < 10) {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color_err, 1);
		} else {
			putText(selectFrame, buf, Point(pixel, line), 1, 1, color, 1);

		}
		line += 20;

		// TODO: use this to change the print color
		if (cs.underVoltageOccured) {
			printf("ERROR     : cam %zu UNDER VOLTAGE OCCURED\n", camIndex);
		}

		if (cs.frequencyCapped) {
			printf("WARNING   : cam %zu ARM FREQUENCY CAPPED\n", camIndex);
		}

		if (cs.throttling) {
			printf("WARNING   : cam %zu THROTTLING\n", camIndex);
		}

		if (cs.softTempLimit) {
			// disabled because occurs to often
			// printf("WARNING   : cam %zu SOFT TEMP LIMIT\n", camIndex);
		}
		// } // if (cs.updated
	} // for (size_t camIndex)

	if (help) {
		putText(selectFrame, "a down rewind 60 frames (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "b      jump to bgr view", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "c      calibrate (toggle)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "d ] >  forward 10 frames (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "f up   forward 60 frames (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "g      show cost table view (toggle)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "h      help (toggle)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "k      show floor overlay (toggle)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "m      manual position", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "n      next view (rotate)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "o      text overlay (toggle)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "p      previous view (rotate)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "q      or esc to exit", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "r      reset to first frame (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "s [ <  rewind 10 frames (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "u      slow or fast viewer refresh rate (toggle)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "w      write config (not implemented)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "x      rewind 1 frame (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "z      forward 1 frame (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
		putText(selectFrame, "space  pause (toggle) (not with camera)", Point(10, line), 1, 1, color, 1);
		line += 20;
	}
}

// show the obstacles in the distorted round viewer
// a "T" is drawn at the found locations
// the "long end" of the T is representing the angle
// the "short end" of the T is representing the minimal distance to the obstacle
void viewer::drawObstacles() {
#ifdef NONO
	float angle, radiusTan;
	Point radius, radiusUp, radiusLeft, radiusRight;
	Point centerPoint = conf->getCenterPoint();
	Scalar color;

	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		angle = obstacles[ii].angle;
		radiusTan = obstacles[ii].radiusTan;
		if( obstacles[ii].size > conf->getObstacle().size ) {
			color = Scalar(255,0,255);
		} else {
			color = Scalar(60,0,60);
		}

		// y-axis is 0 degrees in distorted viewer
		radius.x = centerPoint.x + round( - radiusTan * sin( 2.0f*CV_PI*angle/360.0f ) );
		radius.y = centerPoint.y + round( - radiusTan * cos( 2.0f*CV_PI*angle/360.0f ) );
		// same angle as radius, but a little further away from centerPoint
		radiusUp.x = centerPoint.x + round( - (radiusTan + 50) * sin( 2.0f*CV_PI*angle/360.0f ) );
		radiusUp.y = centerPoint.y + round( - (radiusTan + 50) * cos( 2.0f*CV_PI*angle/360.0f ) );
		// same distance as radius punt, but little distance 90 degrees left of radius point
		radiusLeft.x = radius.x + round( - 20 * sin( 2.0f*CV_PI*(angle+90.0f)/360.0f) );
		radiusLeft.y = radius.y + round( - 20 * cos( 2.0f*CV_PI*(angle+90.0f)/360.0f) );
		// same distance as radius punt, but little distance 90 degrees right of radius point
		radiusRight.x = radius.x + round( - 20 * sin( 2.0f*CV_PI*(angle-90.0f)/360.0f) );
		radiusRight.y = radius.y + round( - 20 * cos( 2.0f*CV_PI*(angle-90.0f)/360.0f) );

		// y-axis is 0 degrees in distorted viewer
		// draw line on obstacle angle starting at closest point obstacle
		line( selectFrame, radius, radiusUp, color, 1);//
		// draw line 90 degrees on angle at closest point obstacle
		line( selectFrame, radiusLeft, radiusRight, color, 1);

		// bounding box around merged obstacle
		// Point leftTop = Point(obstacles[ii].xLeft+centerPoint.x, obstacles[ii].yTop+centerPoint.y);
		// Point rightBottom = Point(obstacles[ii].xRight+centerPoint.x, obstacles[ii].yBottom+centerPoint.y);
		// rectangle(selectFrame, leftTop, rightBottom, Scalar(50,0,50), 1);

	}
#endif
}

// show the obstacles on the rectangular floor
void viewer::drawFloorObstacles(positionStDbl robot) {
	for (size_t cam = 0; cam < 4; cam++) {
		vector<obstacleSt> obstacles = obstDet[cam]->getPositions();

		for (size_t ii = 0; ii < obstacles.size(); ii++) {
			double azimuth = obstacles[ii].azimuth;
			// cam 1 is left of cam 0 = counterclockwise
			azimuth += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated

			// add the angle of the camera itself to the obstacle azimuth
			azimuth += CV_PI * robot.rz / 180.0;

			// x-axis is 0 degrees on rectangular floor
			// there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
			int x = round(robot.x + 0.05 * obstacles[ii].radius * cos(azimuth)); // radius is in mm, while floor pixels are 20mm -> 1/20 = 0.05
			int y = round(robot.y - 0.05 * obstacles[ii].radius * sin(azimuth));

			if (obstacles[ii].size > conf->getBall(obstacleType).pixels) {
				circle(floorFrame, Point(x, y), 8, COLOR_OBSTACLE, -1);
			} else {
				circle(floorFrame, Point(x, y), 4, COLOR_OBSTACLE_SMALL, -1);
			}
		}
	}
}

// show the balls on the rectangular floor
void viewer::drawFloorBalls(positionStDbl robot) {
	for (size_t cam = 0; cam < 4; cam++) {
		vector<ballSt> balls = ballDet[cam]->getPositions();

		for (size_t ii = 0; ii < balls.size(); ii++) {
			double azimuth = balls[ii].azimuth;
			// cam 1 is left of cam 0 = counterclockwise
			azimuth += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated

			// add the angle of the camera itself to the ball azimuth
			azimuth += CV_PI * robot.rz / 180.0;

			// x-axis is 0 degrees on rectangular floor
			// there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
			int x = round(robot.x + 0.05 * balls[ii].radius * cos(azimuth)); // radius is in mm, while floor pixels are 20mm -> 1/20 = 0.05
			int y = round(robot.y - 0.05 * balls[ii].radius * sin(azimuth));

			// TODO: add type to ball that indicates if determined through dewarp or ball pixel size
			int circleType = 1; // open circle
			float maxAngle = -atan(CAMERA_HEIGHT / 6.0); // balls with elevation higher then -7.13 degrees are shown as flying balls
			if (balls[ii].elevation < maxAngle) {
				// ball is on the floor
				circleType = -1; // fill circle
			}

			if (balls[ii].size > conf->getBall(ballType).pixels) {
				circle(floorFrame, Point(x, y), 8, COLOR_BALL, circleType);
			} else {
				circle(floorFrame, Point(x, y), 4, COLOR_BALL_SMALL, circleType);
			}
		}
	}
}

// show the far away balls on the rectangular floor
void viewer::drawFloorBallsFar(positionStDbl robot) {
	for (size_t cam = 0; cam < 4; cam++) {
		vector<ballSt> balls = ballFarDet[cam]->getPositions();

		for (size_t ii = 0; ii < balls.size(); ii++) {
			double azimuth = balls[ii].azimuth;
			// cam 1 is left of cam 0 = counterclockwise
			azimuth += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated

			// add the angle of the camera itself to the ball azimuth
			azimuth += CV_PI * robot.rz / 180.0;

			// x-axis is 0 degrees on rectangular floor
			// there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
			int x = round(robot.x + 0.05 * balls[ii].radius * cos(azimuth)); // radius is in mm, while floor pixels are 20mm -> 1/20 = 0.05
			int y = round(robot.y - 0.05 * balls[ii].radius * sin(azimuth));

			// TODO: add type to ball that indicates if determined through dewarp or ball pixel size
			int circleType = 1; // open circle
			// float maxAngle = -atan(CAMERA_HEIGHT / 6.0); // balls with elevation higher then -7.13 degrees are shown as flying balls
			float maxAngle = -CV_PI * 6.0 / 180.0; // lower then -6 degrees
			// printf("angle %f\n", 180.0*balls[ii].elevation/CV_PI);
			if (balls[ii].elevation < maxAngle) {
				// ball is on the floor
				circleType = -1; // fill circle
			}

			if (balls[ii].size > conf->getBall(ballFarType).pixels) {
				circle(floorFrame, Point(x, y), 8, COLOR_BALL_FAR, circleType);
			} else {
				circle(floorFrame, Point(x, y), 4, COLOR_BALL_FAR_SMALL, circleType);
			}
		}
	}
}

// show the cyans on the rectangular floor
void viewer::drawFloorCyans(positionStDbl robot) {
	int drawRadius = 8; // ball ~ 12cm radius, field 2cm = 1 pixel, obstacle ~ 6 pixels
	Scalar color;

	for (uint ii = 0; ii < cyans.size(); ii++) {
		double azimuth = 2.0f * CV_PI * (cyans[ii].azimuth + robot.rz) / 360.0f;
		double radius = cyans[ii].radius;
		int size = cyans[ii].size;

		// x-axis is 0 degrees on rectangular floor
		// there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
		int x = round(robot.x + radius * cos(azimuth));
		int y = round(robot.y - radius * sin(azimuth));

		if (size > conf->getBall(cyanType).pixels) {
			color = COLOR_CYAN; // high = high probability
		} else {
			color = COLOR_CYAN_SMALL; // low value = low probability
		}
		circle(floorFrame, Point(x, y), drawRadius, color, -1);
	}
}

// show the cyans on the rectangular floor
void viewer::drawFloorMagentas(positionStDbl robot) {
	int drawRadius = 8; // ball ~ 12cm radius, field 2cm = 1 pixel, obstacle ~ 6 pixels
	Scalar color;

	for (uint ii = 0; ii < magentas.size(); ii++) {
		double azimuth = 2.0f * CV_PI * (magentas[ii].azimuth + robot.rz) / 360.0f;
		double radius = magentas[ii].radius;
		int size = magentas[ii].size;

		// x-axis is 0 degrees on rectangular floor
		// there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
		int x = round(robot.x + radius * cos(azimuth));
		int y = round(robot.y - radius * sin(azimuth));

		if (size > conf->getBall(magentaType).pixels) {
			color = COLOR_MAGENTA; // high = high probability
		} else {
			color = COLOR_MAGENTA_SMALL; // low value = low probability
		}
		circle(floorFrame, Point(x, y), drawRadius, color, -1);
	}
}

// show the rz rotated re-mapped view of the robot at x, y in the rectangular field viewer
// angle 0 degrees is on y axis, angle is anti clock wise
void viewer::drawFloorLinePoints(positionStDbl pos, Scalar color) {
	std::vector<angleRadiusSt> floorLinePoints = linePoint->getLinePointsPolar();
	size_t linePoints = floorLinePoints.size();
	ssize_t linePointsUsed = conf->getLine().linePointsNumberMaximal; // TODO: update with number the number from the localization

#ifdef SHOW_WHITE_BALL_PIXELS
	Mat whiteBallCostField = rFloor->getCostField();
	Mat whiteBallFrameTmp = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC1);
#endif

	for (ssize_t ii = linePoints - 1; ii >= 0; ii--) { // draw in reverse order so used pixels will not be hidden between not used pixels
		float angle = floorLinePoints[ii].angle + (2.0f * CV_PI / 360.0f) * pos.rz;
		float radius = floorLinePoints[ii].radius / 20.0; // radius provided in mm, field viewer granularity is 20mm
		int x = round(pos.x + radius * cos(angle));
		int y = round(pos.y - radius * sin(angle)); // top side floor is low y
		// draw positions outside the floor on the border
		if (x < 0) {
			x = 0;
		}
		if (x >= rFloor->getWidth()) {
			x = rFloor->getWidth();
		}
		if (y < 0) {
			y = 0;
		}
		if (y >= rFloor->getHeight()) {
			y = rFloor->getHeight();
		}
		if (ii > linePointsUsed) {
			line(floorFrame, Point(x, y), Point(x, y), COLOR_LINEPOINTS_DARK, 4);
		} else {
			line(floorFrame, Point(x, y), Point(x, y), color, 4);
//			circle(floorFrame, Point(x, y), 4, color, -1);
		}
#ifdef SHOW_WHITE_BALL_PIXELS
		if (whiteBallCostField.at<uchar>(y, x) > 10) {
			printf("INFO      : field x %4d field y %4d camera x %3zu camera y %3zu\n", x, y, floorLinePoints[ii].xCamera, floorLinePoints[ii].yCamera);

		}
		// TODO: only put in the pixels if valid localization
		line(whiteBallFrameTmp, Point(x, y), Point(x, y), 1, 4);
	}

	if (showCostField) {
		whiteBallFrame = whiteBallFrameTmp * 255 + whiteBallCostField;
	} else {
		Mat invertedWhiteBallCostField = 255 - rFloor->getCostField();
		whiteBallFrame = whiteBallFrameTmp.mul(invertedWhiteBallCostField);
#endif
	}
}

// show rz rotated robot at position x, y in the rectangular field viewer
void viewer::drawFloorRobot(positionStDbl pos, Scalar color) {
	int circleRadius = 12; // obstacle / robot ~ 25cm radius, field 2cm = 1 pixel, obstacle ~ 12 pixels
	circle(floorFrame, Point((int) (pos.x + 0.5f), (int) (pos.y + 0.5f)), circleRadius, color, 1);
	int xShooter = round(pos.x + 2.0f * circleRadius * cos(2.0f * CV_PI * pos.rz / 360.0f));
	int yShooter = round(pos.y - 2.0f * circleRadius * sin(2.0f * CV_PI * pos.rz / 360.0f)); // top side floor is low y
	line(floorFrame, Point((int) (pos.x + 0.5f), (int) (pos.y + 0.5f)), Point(xShooter, yShooter), color, 1);
}

void viewer::drawIsolineAzimuth(size_t cam, float az, Vec3b color, int stride)
{
    std::vector<cv::Point> pixels = dewarp[cam]->calcIsolineAzimuth(az, stride);
    drawIsoline(camFrameList[cam], pixels, color);
}

void viewer::drawIsolineElevation(size_t cam, float el, Vec3b color, int stride)
{
    std::vector<cv::Point> pixels = dewarp[cam]->calcIsolineElevation(el, stride);
    drawIsoline(camFrameList[cam], pixels, color);
}

void viewer::drawIsoline(cv::Mat &img, std::vector<cv::Point> const &pixels, Vec3b color)
{
    for (size_t it = 0; it < pixels.size(); ++it)
    {
        // 50% scaling for viewer
        int x = pixels[it].x / 2;
        int y = pixels[it].y / 2;
        img.at<Vec3b>(x, y) = color;
    }
}

void viewer::printStats() {
	if (getTickCount() > nextLogFileTick) {
		// print only every 2 seconds one line
		nextLogFileTick = getTickCount() + (int64) (2.0 * getTickFrequency());

		cout << " frame " << setw(5) << camAnaRecv->getAnaFrameCounter(0);
		cout << " uptime " << setw(6) << fixed << setprecision(1) << prep->getUptime();
		cout << " fps " << setw(4) << fixed << std::setprecision(1) << prep->getFps() << " " << setw(4) << fixed
				<< std::setprecision(1) << loc->getFps();

		uint amount = 0;
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> balls = ballDet[cam]->getPositions();
			for (size_t ii = 0; ii < balls.size(); ii++) {
				if (balls[ii].size >= conf->getBall(ballType).pixels) {
					amount++;
				}
			}
		}
		cout << " balls " << setw(2) << amount;

		amount = 0;
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> balls = ballFarDet[cam]->getPositions();
			for (size_t ii = 0; ii < balls.size(); ii++) {
				if (balls[ii].size >= conf->getBall(ballFarType).pixels) {
					amount++;
				}
			}
		}
		cout << " far " << setw(2) << amount;

		// ball possession is determined by looking at ball pixels before the shooter, which is on camera 0
		cout << " poss " << setw(3) << ballDet[0]->getPossessionPixels();

		amount = 0;
		for (size_t cam = 0; cam < 4; cam++) {
			vector<obstacleSt> obstacle = obstDet[cam]->getPositions();
			for (size_t ii = 0; ii < obstacle.size(); ii++) {
				if (obstacle[ii].size >= conf->getBall(obstacleType).pixels) {
					amount++;
				}
			}
		}
		cout << " obstacles " << setw(2) << amount;

		cout << " points " << dec << setw(3) << linePoint->getLinePointsPolar().size();

		cout << " good " << dec << setw(2) << detPos->getGoodEnoughLoc().goodEnough;

		if (locations.size() > 0) {
			cout << " score " << setw(5) << fixed << std::setprecision(3) << locations[0].score;
			cout << " age " << setw(5) << locations[0].age;
			cout << " active " << setw(5) << locations[0].lastActive << " ";
		}

		cout << " delta " << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(1) << " ";
		cout << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(2) << " ";
		cout << setw(5) << std::setprecision(1) << 1000.0 * camAnaRecv->getRecvDeltaTime(3) << " ms";

		cout << endl;
	}
}

cv::Mat viewer::getRoundFrame() {
	exportMutex.lock();
	cv::Mat retVal = selectFrameExport.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat viewer::getFloorFrame() {
	exportMutex.lock();
	cv::Mat retVal = floorFrameExport.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat viewer::getWhiteBallFrame() {
	exportMutex.lock();
	cv::Mat retVal = whiteBallFrameExport.clone();
	exportMutex.unlock();
	return retVal;
}

void viewer::pixel2camFrame(int &x, int &y, int &cam)
{
    // inverse transformations of placing the 4 camFrames into the viewer
    // TODO check off-by-1 pixel calculations
    int xi = x, yi = y, xo = x, yo = y;
    int cw2 = camHeight / 2; // hmm, was expecting transposed values
    int ch2 = camWidth / 2;
    if ((yi <= ch2) && (xi >= ch2) && (xi <= ch2 + cw2))
    {
        cam = 0;
        yo = 2 * (ch2 - yi);
        xo = 2 * (xi - ch2);
    }
    else if ((yi >= ch2) && (yi <= ch2 + cw2) && (xi <= ch2))
    {
        cam = 1;
        yo = 2 * (ch2 - xi);
        xo = 2 * (ch2 + cw2 - yi);
    }
    else if ((yi >= ch2 + cw2) && (xi >= ch2) && (xi <= ch2 + cw2))
    {
        cam = 2;
        yo = 2 * (yi - ch2 - cw2);
        xo = 2 * (ch2 + cw2 - xi);
    }
    else if ((yi >= ch2) && (yi <= ch2 + cw2) && (xi >= ch2 + cw2))
    {
        cam = 3;
        yo = 2 * (xi - ch2 - cw2);
        xo = 2 * (yi - ch2);
    }
    else
    {
        // other region
        cam = -1;
    }
    // check and return result
    x = xo;
    y = yo;
    if (cam != -1)
    {
        assert(x >= 0);
        assert(y >= 0);
        assert(x <= 2 * cw2);
        assert(y <= 2 * ch2);
    }
}

void viewer::handleClick(int x, int y)
{
    int cam = -1, xp = x, yp = y;
    pixel2camFrame(xp, yp, cam);
    if (cam != -1)
    {
        float floorX = nanf(""), floorY = nanf(""), frontAz = 0.0, frontEl = 0.0;
        bool ok = true;
        int16_t tmpX = 0, tmpY = 0;
        ok = dewarp[cam]->transformFront(yp, xp, frontAz, frontEl);
        if (ok)
        {
            ok = dewarp[cam]->transformFloor(yp, xp, tmpX, tmpY);
            if (ok)
            {
                floorX = 1e-3 * tmpX;
                floorY = 1e-3 * tmpY;
            }
            else
            {
                // floor maps not everywhere defined, continue OK
                ok = true;
            }
        }
        printf("handleClick: cam=%d pixel=(%4d,%4d) floor=(%7.3f,%7.3f) az=%7.3f el=%7.3f\n", cam, xp, yp, floorX, floorY, frontAz, frontEl);
    }
}

void viewer::onMouse(const int event, const int x, const int y, int, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        viewer *self = (viewer *)userdata;
        self->handleClick(x, y);
    }
}

