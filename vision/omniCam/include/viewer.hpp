// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "preprocessor.hpp"
#include "ballDetection.hpp"
#include "ballPossession.hpp"
#include "obstacleDetection.hpp"
#include "determinePosition.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "robotFloor.hpp"
#include "configurator.hpp"

class viewer {
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

	enum viewModeList {
		bgr = 0,
		val,
		hue,
		sat,
		spiralLine,
		lineUnfiltered,
		sunSpot,
		ball,
		cyan,
		magenta,
		obstacleSel,
		last,
		lineErode,
		lineDilate,
		lineSel,
		ballErode,
		ballDilate,
		obstacleErode,
		obstacleDilate
	};
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
	void drawLinePointsAll(cv::Scalar color);
	void drawObstacles();
	void drawFloorBalls(positionStDbl robot);
	void drawFloorCyans(positionStDbl robot);
	void drawFloorMagentas(positionStDbl robot);
	void drawFloorObstacles(positionStDbl robot);
	void drawFloorLinePoints(positionStDbl pos, cv::Scalar color);
	void drawFloorRobot(positionStDbl pos, cv::Scalar color);
	void drawCircleSections(int radius, cv::Scalar color);

public:
	viewer(ballDetection *ballDet, ballPossession *ballPos, configurator *conf, ballDetection *cyan,
			determinePosition *detPos, linePointDetection *linePoint, localization *loc, ballDetection *magenta,
			obstacleDetection *obstDet, preprocessor *prep, robotFloor *rFloor);
	void update();
	cv::Mat getRoundFrame();
	cv::Mat getFloorFrame();
	int getKey() {
		return key;
	}
	bool getPause() {
		return pause;
	}
	void setPause(int pause) {
		this->pause = pause;
	}
	void setViewModeBgr() {
		this->viewMode = bgr;
	}
	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}
};

#endif
