 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef VIEWER_HPP
#define VIEWER_HPP

#include "preprocessor.hpp"
#include "ballDetection.hpp"
#include "ballPossession.hpp"
#include "obstacleDetection.hpp"
#include "determinePosition.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "robotFloor.hpp"
#include "configurator.hpp"

#include <iostream>

#include <mutex>

class viewer
{
private:
	bool busy;
	char key;
	double period; // time between two updates (can be toggled with key 'u')
	int64 nextTick; // time when next viewers are updated
	int64 nextLogFileTick; // time when next line printed in logfile (through console)
	bool textOverlay;
	bool pause;
	bool help;

	int viewMode;
	bool showCostField;
	bool ballView;
	bool cyanView;
	bool magentaView;
	// enum viewModeList { bgr=0, val, hue, sat, spiralLine, lineUnfiltered, lineErode, lineDilate, sunSpot, lineSel, ball, ballErode, ballDilate, cyan, magenta, obstacleSel, obstacleErode, obstacleDilate, last };
	enum viewModeList { bgr=0, val, hue, sat, spiralLine, lineUnfiltered, sunSpot, ball, cyan, magenta, obstacleSel, last, lineErode, lineDilate, lineSel, ballErode, ballDilate, obstacleErode, obstacleDilate };
	std::string selectWindow; // used for round (distorted) camera viewer
	std::string floorWindow; // used for rectangular (floor) viewer
	cv::Mat selectFrame, selectFrameExport; // use for round (distorted) viewer
	cv::Mat floorFrame, floorFrameExport; // used for rectangular (floor) viewer
	cv::Mat floorColorFrame; // used for rectangular (floor) viewer
	cv::Mat linePointsColorFrame; // used to copy the color from for the line points to the round viewer window
	cv::Mat linePointsAllColorFrame; // used to copy the color from for all line points to the round viewer window
	cv::Mat linePointsRejectColorFrame; // used to copy the color from for rejected line points to the round viewer window
	cv::Mat sunSpotContourColorFrame; // used to copy the color from for the contour around the sun spot in the round viewer window
	cv::Mat ballsContourColorFrame; // used to copy the color from for the contour around the ball in the round viewer window
	cv::Mat cyansContourColorFrame; // used to copy the color from for the contour around the cyan in the round viewer window
	cv::Mat magentasContourColorFrame; // used to copy the color from for the contour around the magenta in the round viewer window
	std::vector<cv::Mat> planes;
	std::string viewModeText;
	std::vector<ballSt> balls;
	std::vector<ballSt> cyans;
	std::vector<ballSt> magentas;
	std::vector<detPosSt> locations;
	std::vector<obstacleSt> obstacles;

	std::mutex exportMutex;

	// pointers for access to other classes
	ballDetection *ballDet;
	ballPossession *ballPos;
	configurator *conf;
	ballDetection *cyanDet;
	determinePosition *detPos;
	obstacleDetection *obstDet;
	preprocessor *prep;
	linePointDetection *linePoint;
	localization *loc;
	ballDetection *magentaDet;
	robotFloor *rFloor;

	void roundViewerUpdate();
	void floorUpdate();
	void floorPrintText();

	void printStats();
	void printText();
	void drawRaster();
	void drawLinePointsAll( cv::Scalar color );
	void drawObstacles();
	void drawFloorBalls(positionStDbl robot);
	void drawFloorCyans(positionStDbl robot);
	void drawFloorMagentas(positionStDbl robot);
	void drawFloorObstacles(positionStDbl robot);
	void drawFloorLinePoints(positionStDbl pos, cv::Scalar color);
	void drawFloorRobot(positionStDbl pos, cv::Scalar color);
	void drawCircleSections(int radius, cv::Scalar color);

public:
	viewer(ballDetection *ballDet,
			ballPossession *ballPos,
			configurator *conf,
			ballDetection *cyan,
			determinePosition *detPos,
			linePointDetection *linePoint,
			localization *loc,
			ballDetection *magenta,
			obstacleDetection *obstDet,
			preprocessor *prep,
			robotFloor *rFloor);
	void update();
	cv::Mat getRoundFrame();
	cv::Mat getFloorFrame();
	int getKey() { return key; }
	bool getPause() { return pause; }
	void setPause( int pause ) { this->pause = pause; }
	void setViewModeBgr( ) { this->viewMode = bgr; }
	bool getBusy() { return busy; }
	void setBusy() { busy = true; }
};

#endif
