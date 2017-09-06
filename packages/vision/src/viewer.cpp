 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2017 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Debug tool to show what is actually going
// Using frames in different stages of the process
// Starting from preProcessing upto lineDetection


#include "viewer.hpp"
#include <iomanip>

using namespace cv;
using namespace std;

viewer::viewer(ballDetection *ballDet,
		ballPossession *ballPos,
		configurator *conf,
		ballDetection *cyanDet,
		determinePosition *detPos,
		linePointDetection *linePoint,
		localization *loc,
		ballDetection *magentaDet,
		obstacleDetection *obstDet,
		preprocessor *prep,
		robotFloor *rFloor) {
	this->ballDet = ballDet;
	this->ballPos = ballPos;
	this->conf = conf;
	this->cyanDet = cyanDet;
	this->detPos = detPos;
	this->linePoint = linePoint;
	this->loc = loc;
	this->magentaDet = magentaDet;
	this->obstDet = obstDet;
	this->prep = prep;
	this->rFloor = rFloor;

	exportMutex.lock();
	key = 0;
	textOverlay = true; // default we want the essential information available in the viewers, but it might inconvenient when doing a calibration
	pause = true;
	viewMode = bgr;
	viewModeText = ""; // will be set in show function
	ballView = false; // will be set in show function
	cyanView = false;
	magentaView = false;
	showCostField = false;
	help = false;
	selectWindow = "camera view, press h for help";
	floorWindow = "floor view, press h for help";
	busy = false;
	balls.clear();
	cyans.clear();
	magentas.clear();
	locations.clear();
	obstacles.clear();

	// create once the color image that is later used to copy the line points in the round viewer.
	linePointsColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	linePointsColorFrame = Scalar(255,0,0);

	// create once the color image that is later used to copy all line points in the round viewer.
	linePointsAllColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	linePointsAllColorFrame = Scalar(100,100,100);

	// create once the color image that is later used to copy the rejected line points in the round viewer.
	linePointsRejectColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	linePointsRejectColorFrame = Scalar(0,0,255);

	// create once the color image that is later used to copy the sun spot contour in the round viewer
	sunSpotContourColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	sunSpotContourColorFrame = Scalar(0,0,255);

	// create once the color image that is later used to copy the ball contour in the round viewer
	ballsContourColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	ballsContourColorFrame = COLOR_BALL;

	// create once the color image that is later used to copy the cyan contour in the round viewer
	cyansContourColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	cyansContourColorFrame = COLOR_CYAN;

	// create once the color image that is later used to copy the magenta contour in the round viewer
	magentasContourColorFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	magentasContourColorFrame = COLOR_MAGENTA;

	selectFrame = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);
	selectFrameExport = Mat::zeros(conf->getViewPixels(), conf->getViewPixels(), CV_8UC3);

	// create once the color image that is later used to copy the floor lines in the rectangular (floor) viewer
	floorColorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);
	floorColorFrame = COLOR_LINE;

	floorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);
	floorFrameExport = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);

	period = 0.4;
	nextTick = 0;
	nextLogFileTick = 0;

	exportMutex.unlock();
}

// show both viewers and manage short keys
void viewer::update(){
	if( getTickCount( ) > nextTick ) {
		nextTick = getTickCount( ) + (int64)(period * getTickFrequency( ));

		// acquire the robot locations (with best probability), ball locations and obstacles.
		// do this only once to keep consistency between all users of this information (e.g. drawing in viewers, logging, ..
		balls = ballDet->getPositions();
		cyans = cyanDet->getPositions();
		locations = detPos->getLocList();
		magentas = magentaDet->getPositions();
		obstacles = obstDet->getPositions();

		roundViewerUpdate();
		floorUpdate();

		exportMutex.lock();
		selectFrameExport = selectFrame.clone();
		floorFrameExport = floorFrame.clone();
		exportMutex.unlock();

		if( conf->getGuiEnabled() ) {
			imshow(floorWindow, floorFrame);
			imshow(selectWindow, selectFrame);
		}
	}


	key = waitKey(20);
	switch (key)
	{
	case 'b':
		viewMode = bgr;
		break;
	case 'c':
		conf->setCalibrateMode( ! conf->getCalibrateMode() );
		if( conf->getCalibrateMode() ) {
			conf->setManualMode( true ); // manual mode required for calibration
		} else
		{
			conf->setManualMode( false );
		}
		break;
	case 'h':
		help = ! help;
		break;
	case 'g':
		showCostField = ! showCostField;
		break;
  	case ' ': // space
		pause = ! pause;
		break;
	case 'p':
		viewMode--;
		if( viewMode < 0 ) { viewMode = last - 1; }
		if( viewMode == 3  ) { viewMode = 1; } // skip hue and sat
		break;
	case 'm':
		conf->setManualMode( ! conf->getManualMode() );
		break;
	case 'n':
		viewMode++;
		if( viewMode >= last ) { viewMode = 0; }
		if( viewMode == 2  ) { viewMode = 4; } // skip hue and sat
		break;
	case 'o':
		textOverlay = ! textOverlay;
		break;
	case 'u':
		// move from continues viewing to every x viewing for e.g. slow remote WiFi connection
		if( period > 0.1 ) { period = 0.03; } else { period = 0.4; }
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
void viewer::roundViewerUpdate(){
	ballView = false;
	cyanView = false;
	magentaView = false;
	Mat floorTmp;

	switch(viewMode)
	{
	case bgr:
		selectFrame = prep->getBgr();
		viewModeText = "BGR";
		break;
	case hue:
		split( prep->getHsv(), planes );
	    cvtColor(planes[0], selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Hue";
		break;
	case sat:
 		split( prep->getHsv(), planes );
 		cvtColor(planes[1], selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Sat";
		break;
	case val:
 		split( prep->getHsv(), planes );
 		cvtColor(planes[2], selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Val";
		break;
	case spiralLine:
		cvtColor(linePoint->getSpiralLineFrame(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Spiral Line";
		break;
	case lineUnfiltered:
		cvtColor(linePoint->getLineUnfiltered(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Line Unfiltered";
		break;
	case lineErode:
		cvtColor(linePoint->getLineErode(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Line Erode";
		break;
	case lineDilate:
		cvtColor(linePoint->getLineDilate(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Line Dilate";
		break;
	case sunSpot:
		cvtColor(linePoint->getSunSpot(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Sun Spot";
		break;
	case lineSel:
		cvtColor(linePoint->getLine(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Line";
		break;
	case ball:
		cvtColor(ballDet->getInRangeFrame(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Ball";
		ballView = true;
		break;
	case ballErode:
		cvtColor(ballDet->getErodeFrame(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Ball erode";
		ballView = true;
		break;
	case ballDilate:
		cvtColor(ballDet->getDilateFrame(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Ball dilate";
		ballView = true;
		break;
	case cyan:
		cvtColor(cyanDet->getInRangeFrame(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Cyan";
		cyanView = true;
		break;
	case magenta:
		cvtColor(magentaDet->getInRangeFrame(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Magenta";
		magentaView = true;
		break;
	case obstacleSel:
		cvtColor(obstDet->getObstacle(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Obstacle";
		break;
	case obstacleErode:
		cvtColor(obstDet->getObstacleErode(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Obstacle erode";
		break;
	case obstacleDilate:
		cvtColor(obstDet->getObstacleDilate(), selectFrame, COLOR_GRAY2BGR);
		viewModeText = "Obstacle dilate";
		break;
	default:
		selectFrame = prep->getBgr();
		viewModeText = "BGR";
		break;
	}

	drawRaster();
	if( ! conf->getCalibrateMode() ) {
		// do not display balls and obstacles in calibration mode
	 	linePointsAllColorFrame.copyTo(selectFrame, (linePoint->getLinePointsAllFrame() != 0 ) ); // draw the gray lines showing the part of the spiral
	 	ballsContourColorFrame.copyTo(selectFrame, (ballDet->getViewerContoursFrame() != 0 ) ); // draw the ball contours
	 	cyansContourColorFrame.copyTo(selectFrame, (cyanDet->getViewerContoursFrame() != 0 ) ); // draw the cyan contours
	 	magentasContourColorFrame.copyTo(selectFrame, (magentaDet->getViewerContoursFrame() != 0 ) ); // draw the magenta contours
		drawObstacles();
	 	sunSpotContourColorFrame.copyTo(selectFrame, (linePoint->getSunSpotContours() != 0 ) ); // draw sun spot contour
	 	linePointsRejectColorFrame.copyTo(selectFrame, (linePoint->getLinePointsRejectFrame() != 0 ) ); // draw rejected line points
	 	linePointsColorFrame.copyTo(selectFrame, (linePoint->getLinePointsFrame() != 0 ) ); // draw line points used by determine position
	}
	if( textOverlay ) {
		printText(); // makes image darker when in help mode to make reading easier
	}

	{
		// copy 3 color windows (line, ball and obstacle) and 2 ball possession windows the the corners in the round camera viewer
		// line color window top right
		Mat lineFrame = conf->getLine().window;
		int lineLeft = selectFrame.cols - lineFrame.cols;
		lineFrame.copyTo(selectFrame.colRange(lineLeft, selectFrame.cols).rowRange(0,lineFrame.rows));
		// ball color window bottom left
		Mat ballFrame = conf->getBall( ballType ).window;
		int ballTop = selectFrame.rows - ballFrame.rows;
		ballFrame.copyTo(selectFrame.colRange(0, ballFrame.cols).rowRange(ballTop, selectFrame.rows));
		// obstacle color window bottom right
		Mat obstacleFrame = conf->getObstacle().window;
		int obstacleLeft = selectFrame.cols - obstacleFrame.cols;
		int obstacleTop = selectFrame.rows - obstacleFrame.rows;
		obstacleFrame.copyTo(selectFrame.colRange(obstacleLeft, selectFrame.cols).rowRange(obstacleTop, selectFrame.rows));
		// ball possession bgr bottom left above ball window
		Mat possessionBgrSmall, possessionBgr;
		cvtColor(ballPos->getPossessionHsv(), possessionBgrSmall, COLOR_HSV2BGR); // convert the possession HSV to BGR
		resize(possessionBgrSmall, possessionBgr, Size(), 3.0, 3.0, CV_INTER_LINEAR); // increase to zoom in on ball possession region of interest
		int possessionBgrTop = ballTop - possessionBgr.rows;
		possessionBgr.copyTo(selectFrame.colRange(0, possessionBgr.cols).rowRange(possessionBgrTop, ballTop));
		// ball possession in range frame bottom right above obstacle window
		Mat possessionInRangeFrameSmall, possessionInRangeFrame;
		cvtColor(ballPos->getPossessionInRangeFrame(), possessionInRangeFrameSmall, COLOR_GRAY2BGR);
		resize(possessionInRangeFrameSmall, possessionInRangeFrame, Size(), 3.0, 3.0, CV_INTER_LINEAR); // increase to zoom in on ball possession region of interest
		int possessionInRangeFrameTop = obstacleTop - possessionInRangeFrame.rows;
		int possessionInRangeFrameLeft = selectFrame.cols - possessionInRangeFrame.cols;
		possessionInRangeFrame.copyTo(selectFrame.colRange(possessionInRangeFrameLeft, selectFrame.cols).rowRange(possessionInRangeFrameTop, obstacleTop));
	}
}

// update data for rectangular robot floor viewer
void viewer::floorUpdate() {
	floorFrame = Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC3);
	floorFrame = COLOR_GRASS;

	if( showCostField ) {
		vector<Mat> floorTmp;
		floorTmp.push_back(Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC1));
		floorTmp.push_back(rFloor->getCostField());
		floorTmp.push_back(Mat::zeros(rFloor->getHeight(), rFloor->getWidth(), CV_8UC1));
	    cv::merge(floorTmp, floorFrame);
	} else {
		floorColorFrame.copyTo(floorFrame, (rFloor->getRefField() != 0 ) ); // virtual field used for human interpretation
	}

	if( ! conf->getCalibrateMode() ) {
		// do not display balls, obstacles and search information in calibration mode

		if( locations.size() > 0 ) {
			// so we have a lock, get current position
			positionStDbl robot = locations[0].pos; // use the "best" found location as the projected robot position
			drawFloorObstacles( robot );
			drawFloorCyans( robot );
			drawFloorMagentas( robot );
			drawFloorBalls( robot );
			for( unsigned int ii=0; ii<locations.size(); ii++) {
				if( ii == 0 ) {
					if( locations[ii].lastActive < 5 ) {
						drawFloorLinePoints(locations[ii].pos, COLOR_LINEPOINTS );
						drawFloorRobot(locations[ii].pos, COLOR_ROBOT );
					} else {
						drawFloorLinePoints(locations[ii].pos, COLOR_LINEPOINTS_DARK );
						drawFloorRobot(locations[ii].pos, COLOR_ROBOT_DARK );
					}
				}
			}
		}
	}
	if( textOverlay ) { floorPrintText(); }
}

// print the text no the rectangular robot floor viewer
void viewer::floorPrintText() {
	char buf[256];
	char *bufp = buf;
	int line = 20;
	Scalar color(0,255,255); // yellow
	std::stringstream strForm;
	strForm << "Falcon " << conf->getRobot();
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	strForm << "Frame " << setw(6) << prep->getCount();
	strForm << " local " << setw(6) << loc->getCount();
	strForm << " uptime " << setw(6) << fixed << std::setprecision(1) << prep->getUptime();
	strForm << " fps " << setw(4) << fixed << std::setprecision(1)  << prep->getFps() << " " << loc->getFps();
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	uint amount = 0;
	for( uint ii = 0; ii < balls.size(); ii++ ) {
		if( balls[ii].size >= conf->getBall( ballType ).size ) {	amount++; }
	}
	strForm << "Balls" << setw(2) << amount << " possession " << ballPos->getPossessionPixels();
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	amount = 0;
	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		if( obstacles[ii].size >= conf->getObstacle().size ) { amount++; }
	}
	strForm << "Obstacles " << amount;
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	// print localization results
	strForm << "Line Points " << setw(3) << linePoint->getLinePointsRadianMeter().size();
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=40; strForm.str("");
	strForm << "Score ";
	for( size_t ii=0; ii<locations.size(); ii++ ) {
		strForm << setw(5) << fixed << std::setprecision(3) << locations[ii].score << " ";
	}
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	strForm << "Age   ";
	for( size_t ii=0; ii<locations.size(); ii++ ) {
		strForm << setw(5) << locations[ii].age << " ";
	}
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	strForm << "Active ";
	for( size_t ii=0; ii<locations.size(); ii++ ) {
		strForm << setw(5) << locations[ii].lastActive << " ";
	}
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");


	strForm << "Tries  ";
	for( size_t ii=0; ii<locations.size(); ii++ ) {
		strForm << setw(5) << locations[ii].numberOfTries << " ";
	}
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	strForm << "Pixels ";
	for( size_t ii=0; ii<locations.size(); ii++ ) {
		strForm << setw(5) << locations[ii].amountOnFloor << " ";
	}
	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=40; strForm.str("");

	// if( ! conf->getCalibrateMode() ) {
	// 	strForm << "Compass "<< fixed << std::setprecision(0)  << compass->get_rotation();
	//	putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
	// }
	if( locations.size() > 0 ) {
		putText(floorFrame, "Position    x       y        rz", Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
		strForm << "Vision  " << std::setprecision(1) << setw(6) << locations[0].pos.x << " " << setw(6) << locations[0].pos.y << " " << setw(6) << locations[0].pos.rz;
		putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

		detPosSt locGood = detPos->getGoodEnoughLocExport( );
		if( locGood.goodEnough ) {
			strForm << "Good    " << std::setprecision(1) << setw(6) << locGood.pos.x << " " << setw(6) << locGood.pos.y << " " << setw(6) << locGood.pos.rz;
			putText(floorFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
		}

		detPosSt locRos = detPos->getGoodEnoughLocExportRos( );
		if( locRos.goodEnough ) {
			double angleDeg = locRos.pos.rz*180.0/CV_PI;
			if( angleDeg > 180.0 ) { angleDeg -= 360; }
			sprintf(bufp, "Ros  %4.1f cm y %4.1f cm %4.1f deg", 100.0*locRos.pos.y, -100.0*locRos.pos.x, angleDeg);
			putText(floorFrame, buf, Point(10,line), 1, 1, color, 1); line+=20;
		}
	}

	if( conf->getManualMode()) { putText(floorFrame, "Manual mode", Point(10,line), 1, 1, color, 1); line+=20; }
	if( conf->getCalibrateMode()) { putText(floorFrame, "Calibration mode", Point(10,line), 1, 1, color, 1); line+=20; }
}

void viewer::printText()
{
	int line = 20;
	Scalar color(0,255,255); // yellow
	std::stringstream strForm;
	if(help)
	{
		selectFrame = selectFrame * 0.5; // make background darker to make help text easier to read
	}
	strForm << "Falcon " << conf->getRobot();
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	strForm << "Frame " << setw(6) << prep->getCount();
	strForm << " local " << setw(6) << loc->getCount();
	strForm << " uptime " << setw(6) << fixed << std::setprecision(1) << prep->getUptime();
	strForm << " fps " << setw(4) << fixed << std::setprecision(1)  << prep->getFps() << " " << loc->getFps();
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
	putText(selectFrame, viewModeText, Point(10,line),1,1,color,1); line+=20;

	line+=20;

	uint amount = 0;
	for( size_t ii = 0; ii < balls.size(); ii++ ) {
		if( balls[ii].size >= conf->getBall( ballType ).size ) { amount++; }
	}
	strForm << "Balls" << setw(2) << amount << " possession " << ballPos->getPossessionPixels();
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	amount = 0;
	for( size_t ii = 0; ii < cyans.size(); ii++ ) {
		if( cyans[ii].size >= conf->getBall( cyanType ).size ) { amount++; }
	}
	strForm << "Cyans" << setw(2) << amount;
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	amount = 0;
	for( size_t ii = 0; ii < magentas.size(); ii++ ) {
		if( magentas[ii].size >= conf->getBall( magentaType ).size ) { amount++; }
	}
	strForm << "Magentas" << setw(2) << amount;
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	amount = 0;
	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		if( obstacles[ii].size >= conf->getObstacle().size ) { amount++; }
	}
	strForm << "Obstacles " << amount;
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	strForm << "Line Points " << setw(3) << linePoint->getLinePointsRadianMeter().size();
	putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");

	if( ballView || viewMode == bgr ) {
		line+=20;
		if( balls.size() > 0 ) {
			putText(selectFrame, "Ball size angle radTan radLin radRos", Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
		} else {
			line+=20;
		}
		for( size_t ii = 0; ii < balls.size(); ii++ ) {
			if( balls[ii].size >= conf->getBall( ballType ).size ) {
				if( ii < 3 ) {
					strForm << setw(6) << balls[ii].size << " ";
					strForm << setw(6) << std::setprecision(1) << balls[ii].angle << " ";
					strForm << setw(6) << balls[ii].radiusTan << " ";
					strForm << setw(6) << balls[ii].radiusLin << " ";
					strForm << setw(6) << setprecision(2) << (0.02f * balls[ii].radiusLin);
					putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
				}
			}
		}
	}
	if( cyanView || viewMode == bgr ) {
		line+=20;
		if( cyans.size() > 0 ) {
			putText(selectFrame, "Cyan size angle radTan radLin radRos", Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
		} else {
			line+=20;
		}
		for( size_t ii = 0; ii < cyans.size(); ii++ ) {
			if( cyans[ii].size >= conf->getBall( cyanType ).size ) {
				if( ii < 3 ) {
					strForm << setw(6) << cyans[ii].size << " ";
					strForm << setw(6) << std::setprecision(1) << cyans[ii].angle << " ";
					strForm << setw(6) << cyans[ii].radiusTan << " ";
					strForm << setw(6) << cyans[ii].radiusLin << " ";
					strForm << setw(6) << setprecision(2) << (0.02f * cyans[ii].radiusLin);
					putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
				}
			}
		}
	}
	if( magentaView || viewMode == bgr ) {
		line+=20;
		if( magentas.size() > 0 ) {
			putText(selectFrame, "Magenta size angle radTan radLin radRos", Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
		} else {
			line+=20;
		}
		for( size_t ii = 0; ii < magentas.size(); ii++ ) {
			if( magentas[ii].size >= conf->getBall( magentaType ).size ) {
				if( ii < 3 ) {
					strForm << setw(6) << magentas[ii].size << " ";
					strForm << setw(6) << std::setprecision(1) << magentas[ii].angle << " ";
					strForm << setw(6) << magentas[ii].radiusTan << " ";
					strForm << setw(6) << magentas[ii].radiusLin << " ";
					strForm << setw(6) << setprecision(2) << (0.02f * magentas[ii].radiusLin);
					putText(selectFrame, strForm.str(), Point(10,line), 1, 1, color, 1); line+=20; strForm.str("");
				}
			}
		}
	}

	if(help)
	{
		putText(selectFrame, "a down rewind 60 frames (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "b      jump to bgr view",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "c      calibrate (toggle)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "d ] >  forward 10 frames (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "f up   forward 60 frames (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "g      show cost table view (toggle)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "h      help (toggle)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "m      manual position",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "n      next view (rotate)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "o      text overlay (toggle)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "p      previous view (rotate)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "q      or esc to exit",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "r      reset to first frame (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "s [ <  rewind 10 frames (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "u      slow or fast viewer refresh rate (toggle)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "w      write config (not implemented)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "x      rewind 1 frame (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "z      forward 1 frame (not with camera)",Point(10,line),1,1,color,1); line+=20;
		putText(selectFrame, "space  pause (toggle) (not with camera)",Point(10,line),1,1,color,1); line+=20;
	}
}

void viewer::drawCircleSections(int radius, cv::Scalar color) {
	Point centerPoint = conf->getCenterPoint();
	line( selectFrame, Point(centerPoint.x-20,centerPoint.y-radius), Point(centerPoint.x+20,centerPoint.y-radius), color, 1, CV_AA);
	line( selectFrame, Point(centerPoint.x-20,centerPoint.y+radius), Point(centerPoint.x+20,centerPoint.y+radius), color, 1, CV_AA);
	line( selectFrame, Point(centerPoint.x-radius,centerPoint.y-20), Point(centerPoint.x-radius,centerPoint.y+20), color, 1, CV_AA);
	line( selectFrame, Point(centerPoint.x+radius,centerPoint.y-20), Point(centerPoint.x+radius,centerPoint.y+20), color, 1, CV_AA);
}
// show the raster in the distorted round viewer
void viewer::drawRaster( ){
	Point centerPoint = conf->getCenterPoint();
	int viewPixels = conf->getViewPixels();
	int robot = conf->getRobot();
	Scalar rasterColor(255,255,255);
	Scalar yellow(0,255,255);
	Scalar darkYellow(0,150,150);
	Scalar grey(100,100,100);
	Scalar lightGrey(150,150,150);
	Scalar red(0,0,255);
	Scalar blue(255,0,0);

	if( conf->getCalibrateMode() && ( viewMode == bgr || viewMode == hue || viewMode == sat || viewMode == val ||
			viewMode == lineUnfiltered || viewMode == lineErode || viewMode == lineDilate || viewMode == sunSpot ||
			viewMode == lineSel ) ) {
		// exact center pixel = 350,350 (instead of 359.5,359,5)
		// circle( selectFrame, centerPoint, conf->getCameraRadiusMin(), RASTERCOLOR, 1, CV_AA); // grey
		// circle( selectFrame, centerPoint, conf->getCameraRadiusMax(), RASTERCOLOR, 1, CV_AA);
		if( robot == 1 ) { // looks pretty much the same as robot 4, use same values, except x-bias and y-bias
			// robot 1 has 3 washers equally spaced and the pin is pretty much in the middle but has a slight bend at the end
			// focus camera 1 should be 100 instead of 90
			// calibrated April 2, 2015, pretty good at least 2 of 4 axis
			// checked June 25, 2015, pretty good on 3 of 4 axis, no change = no yaml update required
			//                  25  50  75  100  125  150  175  200  225  250  275  300  325  350  375  400  425  450
			int calPoints[] = { 27, 56, 83, 113, 141, 167, 191, 213, 233, 251, 266, 281, 293, 304, 314, 323, 331, 338 };
			for ( uint ii = 0; ii < sizeof(calPoints)/sizeof(calPoints[0]); ii++ ) {
				if( (ii+1) % 4 == 0 ) {
					drawCircleSections( calPoints[ii], red );
				} else {
					drawCircleSections( calPoints[ii], yellow );
				}
			}
		} else if ( robot == 2 ) {
			// robot 2 has 3 washers equally spaced and the pin is exactly straight aligned
			// calibrated April 2, 2015, pretty good at least 2 of 4 axis
			// Checked June 25, 2015, still very good and no adjustments required
			// re-calibrated September 8, 2016
			// focus adjust, calPoints adjusted, 4 out of 4 decent (probably camera tilted a little)
			//                  25  50  75  100  125  150  175  200  225  250  275  300  325  350  375  400  425  450
			int calPoints[] = { 32, 64, 92, 119, 147, 172, 195, 217, 236, 253, 268, 282, 294, 306, 315, 324, 332, 338 };
			for ( uint ii = 0; ii < sizeof(calPoints)/sizeof(calPoints[0]); ii++ ) {
				if( (ii+1) % 4 == 0 ) {
					drawCircleSections( calPoints[ii], red );
				} else {
					drawCircleSections( calPoints[ii], yellow );
				}
			}
		} else if ( robot == 3 ) {
			// calibrated June 25, 2015, difficult to get it correct for more then 2 dimensions, requires yaml update
			// this camera should be mechanically re-assembled to get at least 2 dimensions right
			// re-calibrated September 8, 2016
			// focus adjust, calPoints adjusted, looks pretty good now, 2 out of 4 very good, other 2 decent
			//                  25  50  75  100  125  150  175  200  225  250  275  300  325  350  375  400  425
			int calPoints[] = { 32, 64, 93, 120, 147, 173, 195, 219, 237, 254, 270, 283, 296, 306, 315, 324, 331 };
			for ( uint ii = 0; ii < sizeof(calPoints)/sizeof(calPoints[0]); ii++ ) {
				if( (ii+1) % 4 == 0 ) {
					drawCircleSections( calPoints[ii], red );
				} else {
					drawCircleSections( calPoints[ii], yellow );
				}
			}
		} else if( robot == 4 ) {
			// calibrated April 2, 2015, don't know about the washers, it seems this one was swapped with robot 6
			// checked June 25, 2015, okay, no changes = no yaml update required
			// the spike in the middle is 100% aligned
			// during calibration, the physical robot is placed a exact center of chessboard cross!
			// this is used to set the x optBias and y optBias
			// The numbers below match very good on the Y axis
			// September 8, 2016, re-mounted the mirror with glue
			// focus adjustment, exact spike center adjusted, calPoints adjusted, looks pretty good now, 2 out of 4 very good, other 2 decent
			//                  25  50  75  100  125  150  175  200  225  250  275  300  325  350  375  400  425  450
			int calPoints[] = { 32, 62, 90, 118, 145, 171, 194, 216, 235, 252, 267, 281, 293, 304, 313, 321, 330, 338 };
			for ( uint ii = 0; ii < sizeof(calPoints)/sizeof(calPoints[0]); ii++ ) {
				if( (ii+1) % 4 == 0 ) {
					drawCircleSections( calPoints[ii], red );
				} else {
					drawCircleSections( calPoints[ii], yellow );
				}
			}
		} else if( robot == 5 ) {
			// calibrated April 2, 2015
			// checked June 25 2015, okay, no change = no yaml update required
			// Robot 5 was one of the early optical adjusted and does not have the spike 90 degrees in the image
			// did not update the numbers below, only top y correct, bottom y needs some adjustment, maybe recalibrate
			// one day with the spike aligned
			// September 8, 2016, re-mounted the mirror with glue, unfortunately the mirror is slightly tilted
			// focus adjustment, not found a way to get a proper correction, abort
			//                  25  50  75  100  125  150  175  200  225  250  275  300  325  350  375  400  425  450
			int calPoints[] = { 27, 55, 82, 112, 140, 167, 190, 213, 231, 248, 264, 278, 290, 301, 311, 320, 329, 335 };
			for ( uint ii = 0; ii < sizeof(calPoints)/sizeof(calPoints[0]); ii++ ) {
				if( (ii+1) % 4 == 0 ) {
					drawCircleSections( calPoints[ii], red );
				} else {
					drawCircleSections( calPoints[ii], yellow );
				}
			}
		} else if ( robot == 6 ) {
			// calibrated April 2, 2015, note it looks like there is quite some rz error
			// September 8, 2016, re-mounted the mirror with glue
			// focus adjustment, exact spike center adjusted, calPoints adjusted, looks pretty good now, 2 out of 4 very good, other 2 decent
			//                  25  50  75  100  125  150  175  200  225  250  275  300  325  350  375  400  425  450
			int calPoints[] = { 26, 56, 82, 111, 138, 165, 189, 210, 230, 247, 263, 277, 289, 300, 311, 320, 328, 335 };
			for ( uint ii = 0; ii < sizeof(calPoints)/sizeof(calPoints[0]); ii++ ) {
				if( (ii+1) % 4 == 0 ) {
					drawCircleSections( calPoints[ii], red );
				} else {
					drawCircleSections( calPoints[ii], yellow );
				}
			}
		}
	}
	// Point rectCorner(100,100);
	// rectangle(frameLineDots, center2d - rectCorner, center2d + rectCorner, DRAW0, 1, CV_AA);
	line( selectFrame, Point(0,centerPoint.y), Point(viewPixels-1,centerPoint.y), rasterColor, 1, CV_AA);
	line( selectFrame, Point(centerPoint.x,0), Point(centerPoint.x,viewPixels-1), rasterColor, 1, CV_AA);
	// draw the robot with shooter for reference
	circle(selectFrame, centerPoint, 20, blue, 1, CV_AA);
	if( ! conf->getCalibrateMode( ) ) {
		circle(selectFrame, centerPoint, conf->getCameraRadiusObstacleMin(), grey, 1, 4 );
		circle(selectFrame, centerPoint, conf->getCameraRadiusObstacleMax(), grey, 1, 4 );
		circle(selectFrame, centerPoint, conf->getCameraRadiusLineMax(), lightGrey, 1, 4 );
		circle(selectFrame, centerPoint, conf->getCameraRadiusBallMax(), darkYellow, 1, 4 );
	}
	line(selectFrame, centerPoint, Point(centerPoint.x,centerPoint.y-40), blue, 1, CV_AA);
}

// show the obstacles in the distorted round viewer
// a "T" is drawn at the found locations
// the "long end" of the T is representing the angle
// the "short end" of the T is representing the minimal distance to the obstacle
void viewer::drawObstacles( ){
	float angle, radiusTan;
	Point radius, radiusUp, radiusLeft, radiusRight;
	Point centerPoint = conf->getCenterPoint();
	Scalar color;

	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		angle = obstacles[ii].angle;
		radiusTan = obstacles[ii].radiusTan;
		if( obstacles[ii].size > conf->getObstacle().size ) {
			color =	Scalar(255,0,255);
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
		line( selectFrame, radius, radiusUp, color, 1); //
		// draw line 90 degrees on angle at closest point obstacle
		line( selectFrame, radiusLeft, radiusRight, color, 1);

		// bounding box around merged obstacle
		// Point leftTop = Point(obstacles[ii].xLeft+centerPoint.x, obstacles[ii].yTop+centerPoint.y);
		// Point rightBottom = Point(obstacles[ii].xRight+centerPoint.x, obstacles[ii].yBottom+centerPoint.y);
		// rectangle(selectFrame, leftTop, rightBottom, Scalar(50,0,50), 1);

	}
}

// show the obstacles on the rectangular floor
void viewer::drawFloorObstacles(positionStDbl robot){
	Point pos, pos2;
	int circleRadius = 12; // obstacle / robot ~ 25cm radius, field 2cm = 1 pixel, obstacle ~ 12 pixels
	Scalar color;

	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		double angle = 2.0f*CV_PI*( obstacles[ii].angle + robot.rz ) / 360.0f;

		// do 2 angle/rad to x,y conversions, the first to determine if the obstacle is inside the field, and the
		// second to calculate the center of the circle

		float radiusLin = obstacles[ii].radiusLin;

		// x-axis is 0 degrees on rectangular floor
		pos.x = round( robot.x + radiusLin * cos( angle ) );
		pos.y = round( robot.y - radiusLin * sin( angle ) );

		// use size of obstacle to determine if obstacle is important or just noise
		// because of black floor border, check also if object is within field lines
		int size = obstacles[ii].size;
		if( size > conf->getObstacle().size && rFloor->inField( pos ) )  {
			color = COLOR_OBSTACLE; // high value = high probability
		} else {
			color = COLOR_OBSTACLE_SMALL; // low value = low probability
		}

		// the obstacle radius is defined as closest point to obstacle, compensate for this with adding circleRadius
		float radiusLin2 = radiusLin + circleRadius;

		// x-axis is 0 degrees on rectangular floor
		pos2.x = round( robot.x + radiusLin2 * cos( angle ) );
		pos2.y = round( robot.y - radiusLin2 * sin( angle ) );

		circle(floorFrame, pos2, circleRadius, color, -1);
	}
}

// show the balls on the rectangular floor
void viewer::drawFloorBalls(positionStDbl robot){
	int radius = 8; // ball ~ 12cm radius, field 2cm = 1 pixel, obstacle ~ 6 pixels
	Scalar color;

	for( uint ii = 0; ii < balls.size(); ii++ ) {
		double angle = 2.0f*CV_PI*( balls[ii].angle + robot.rz ) / 360.0f;
		double radiusLin = balls[ii].radiusLin;
		int size = balls[ii].size;

		// x-axis is 0 degrees on rectangular floor
		// there is no need to add a ballRadius to radiusLin (ball etdge) because in field view the radiusLin is very close to the center
		int x = round( robot.x + radiusLin * cos( angle ) );
		int y = round( robot.y - radiusLin * sin( angle ) );

		if( size > conf->getBall( ballType ).size ) {
			color = COLOR_BALL; // high = high probability
		} else {
			color = COLOR_BALL_SMALL; // low value = low probability
		}
		circle(floorFrame, Point(x,y), radius, color, -1);
	}
}

// show the cyans on the rectangular floor
void viewer::drawFloorCyans(positionStDbl robot){
	int radius = 8; // ball ~ 12cm radius, field 2cm = 1 pixel, obstacle ~ 6 pixels
	Scalar color;

	for( uint ii = 0; ii < cyans.size(); ii++ ) {
		double angle = 2.0f*CV_PI*( cyans[ii].angle + robot.rz ) / 360.0f;
		double radiusLin = cyans[ii].radiusLin;
		int size = cyans[ii].size;

		// x-axis is 0 degrees on rectangular floor
		// there is no need to add a ballRadius to radiusLin (ball etdge) because in field view the radiusLin is very close to the center
		int x = round( robot.x + radiusLin * cos( angle ) );
		int y = round( robot.y - radiusLin * sin( angle ) );

		if( size > conf->getBall( cyanType ).size ) {
			color = COLOR_CYAN; // high = high probability
		} else {
			color = COLOR_CYAN_SMALL; // low value = low probability
		}
		circle(floorFrame, Point(x,y), radius, color, -1);
	}
}

// show the cyans on the rectangular floor
void viewer::drawFloorMagentas(positionStDbl robot){
	int radius = 8; // ball ~ 12cm radius, field 2cm = 1 pixel, obstacle ~ 6 pixels
	Scalar color;

	for( uint ii = 0; ii < magentas.size(); ii++ ) {
		double angle = 2.0f*CV_PI*( magentas[ii].angle + robot.rz ) / 360.0f;
		double radiusLin = magentas[ii].radiusLin;
		int size = magentas[ii].size;

		// x-axis is 0 degrees on rectangular floor
		// there is no need to add a ballRadius to radiusLin (ball etdge) because in field view the radiusLin is very close to the center
		int x = round( robot.x + radiusLin * cos( angle ) );
		int y = round( robot.y - radiusLin * sin( angle ) );

		if( size > conf->getBall( magentaType ).size ) {
			color = COLOR_MAGENTA; // high = high probability
		} else {
			color = COLOR_MAGENTA_SMALL; // low value = low probability
		}
		circle(floorFrame, Point(x,y), radius, color, -1);
	}
}

// show the rz rotated remapped view of the robot at x, y in the rectangular field viewer
// angle 0 degrees is on y axis, angle is anti clock wise
void viewer::drawFloorLinePoints( positionStDbl pos, Scalar color ){
	std::vector<angleRadiusSt> floorLinePoints = linePoint->getLinePointsRadianMeter();
	size_t linePoints = floorLinePoints.size();
	// there might be more line points, but show only the amount that are used
	if( linePoints > conf->getLine().linePointsNumber ) { linePoints = conf->getLine().linePointsNumber; }
	for( size_t ii = 0; ii < linePoints; ii++ ) {
		float angle = floorLinePoints[ii].angle + (2.0f*CV_PI/360.0f) * pos.rz;
		float radius = floorLinePoints[ii].radius; // already converted from distorted tan view to linear view
		int x = round( pos.x + radius * cos(angle) );
		int y = round( pos.y - radius * sin(angle) ); // top side floor is low y
		// draw positions outside the floor on the border
		if( x < 0 ) { x = 0; }
		// APOX todo: the following sizes should be resolved from the Configurator
		if( x >= rFloor->getWidth() ) { x = rFloor->getWidth(); }
		if( y < 0 ) { y = 0; }
		if( y >= rFloor->getHeight() ) { y = rFloor->getHeight(); }
		line( floorFrame, Point(x,y), Point(x,y), color, 2);
	}
}

// show rz rotated robot at position x, y in the rectangular field viewer
void viewer::drawFloorRobot( positionStDbl pos, Scalar color ){
	int circleRadius = 12; // obstacle / robot ~ 25cm radius, field 2cm = 1 pixel, obstacle ~ 12 pixels
	circle(floorFrame, Point((int)(pos.x+0.5f), (int)(pos.y+0.5f)), circleRadius, color, 1);
	int xShooter = round( pos.x + 2.0f * circleRadius * cos(2.0f*CV_PI*pos.rz/360.0f) );
	int yShooter = round( pos.y - 2.0f * circleRadius * sin(2.0f*CV_PI*pos.rz/360.0f) ); // top side floor is low y
	line(floorFrame, Point((int)(pos.x+0.5f),(int)(pos.y+0.5f)), Point(xShooter,yShooter), color, 1);
}


void viewer::printStats() {
	if( getTickCount( ) > nextLogFileTick ) {
		// print only every 2 seconds one line
		nextLogFileTick = getTickCount( ) + (int64)(2.0 * getTickFrequency( ));

		cout << " frame " << setw(5) << prep->getCount();
		cout << " local " << setw(5) << loc->getCount();
		cout << " uptime " << setw(6) << fixed << setprecision(1) << prep->getUptime();
		cout << " fps " << setw(4) << fixed << std::setprecision(1) << prep->getFps() << " " << setw(4) << fixed << std::setprecision(1) << loc->getFps();

		uint amount = 0;
		for( uint ii = 0; ii < balls.size(); ii++ ) {
			if( balls[ii].size >= conf->getBall( ballType ).size ) { amount++; }
		}
		cout << " balls " << setw(2) << amount;
		cout << " poss " << setw(3) << ballPos->getPossessionPixels();

		amount = 0;
		for( uint ii = 0; ii < obstacles.size(); ii++ ) {
			if( obstacles[ii].size >= conf->getObstacle().size ) { amount++; }
		}
		cout << " obstacles " << setw(2) << amount;

		cout << " points " << dec << setw(3) << linePoint->getLinePointsRadianMeter().size();

		cout << " good " << dec << setw(2) << detPos->getGoodEnoughLoc().goodEnough;

		if( locations.size() > 0 ) {
			cout << " score " << setw(5) << fixed << std::setprecision(3) << locations[0].score;
			cout << " age " << setw(5) << locations[0].age;
			cout << " active " << setw(5) << locations[0].lastActive << " ";
		}

		cout << endl;
	}
}

cv::Mat viewer::getRoundFrame( ) {
	exportMutex.lock();
	cv::Mat retVal = selectFrameExport.clone();
	exportMutex.unlock();
	return retVal;
}

cv::Mat viewer::getFloorFrame( ) {
	exportMutex.lock();
	cv::Mat retVal = floorFrameExport.clone();
	exportMutex.unlock();
	return retVal;
}
