// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CONFIGURATOR_HPP
#define CONFIGURATOR_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

#include "cameraControl.hpp"

typedef struct {
	float x;
	float y;
	float rz;
	float angle; // in degrees (0 to 360)
	float radiusTan;
	float radiusLin;
	float score;
} positionStFl;

typedef struct {
	double x;
	double y;
	double rz;
} positionStDbl;

typedef struct {
	int x;
	int y;
	int rz;
} positionStInt;

struct detPosSt {
	positionStDbl pos;
	bool goodEnough;
	double score;
	int lastActive; // last time (in frames) location was found
	int age; // life time in amount of frames
	int amountOnFloor;
	int amountOffFloor;
	int numberOfTries;
	bool operator<(const detPosSt &val) const {
		// sorting this struct is performed on score (lower is better) + lastActive (penalty) - age (improve)
		return (score + lastActive * 0.01f - age * 0.00001f) < (val.score + val.lastActive * 0.01f - val.age * 0.00001f);
	}
};

typedef struct {
	int xLeft;
	int xRight;
	int yTop;
	int yBottom;
} floorSizeSt;

typedef struct {
	int min;
	int max;
	int center;
	int delta;
} rangeStruct;

typedef struct {
	rangeStruct hue;
	rangeStruct sat;
	rangeStruct val;
	rangeStruct calVal;
	cv::Mat window;
	int linePointsMinimal;
	size_t linePointsNumber;
	int lineRejectWidth;
	int lineHoleWidth;
	int erodeSlider;
	cv::Mat erode;
	int dilateSlider;
	cv::Mat dilate;
	int score;
	int size;
	int sizeMax;
	int sunSpotWidth;
	int sunSpotHeight;
	int sunSpotRatio;
	int goalRearMask; // only used by robot1
	int goalRearDistance; // only used by robot1
	int goalRearAngle; // only used by robot1
	int goalRearWatchdog; // only used by robot1
} colStruct;

typedef struct {
	rangeStruct hue;
	rangeStruct sat;
	rangeStruct val;
	rangeStruct calVal;
	int yOffset;
	int width;
	int height;
	unsigned int threshold;
} possessionStruct;

typedef struct {
	int xStep;
	int yStep;
	int rzStep;
	int maxCount;
	int score;
} solverSt;

typedef struct {
	double xStep;
	double yStep;
	double rzStep;
	int maxCount;
	double score;
} solverStDbl;

typedef struct {
	double score;
	int amountOnFloor;
	int amountOffFloor;
	bool positionFound;
	double confidenceLevel;
} scoreStruct;

typedef struct {
	double pixelPow1;
	double pixelPow2;
	double inSideTan;
	double outSideTan;
} deWarpSt;

typedef struct {
	int keepWatchdog;
	double xDeltaKeepThreshold;
	double yDeltaKeepThreshold;
	double rzDeltaKeepThreshold;
	int newMinAge;
	double newThreshold;
	double angleToPixelRatio; // error = xDelta + yDelta + angleToPixelRatio * rzDelta
} goodEnoughSt;

typedef struct {
	int frequency;
	std::string ip;
	int port;
} multicastSt;

typedef struct {
	int x;
	int y;
	int rz;
	int rzBall;
	int rzObstacle;
} exportOffsetIntSt;

typedef struct {
	double x;
	double y;
	double rz;
	double rzBall;
	double rzObstacle;
} exportOffsetDblSt;

enum ballDetectionTypes {
	ballType = 0, cyanType, magentaType
};

#define COLOR_GRASS Scalar(0, 50 ,0)
#define COLOR_LINE Scalar(150, 150, 150)
#define COLOR_LINEPOINTS Scalar(255, 255, 255)
#define COLOR_LINEPOINTS_DARK Scalar(255, 255, 255)
#define COLOR_LINEPOINTS_OLD Scalar(127, 127 ,127)
#define COLOR_LINEPOINTS_INVALID Scalar(51, 51, 255);
#define COLOR_ROBOT Scalar(0, 0, 255)
#define COLOR_ROBOT_DARK Scalar(0, 0, 80)
#define COLOR_ROBOT_OLD Scalar(51, 153, 255);
#define COLOR_ROBOT_BALLPOSSESSION Scalar(0, 255, 255)
#define COLOR_OBSTACLE Scalar(0, 0 ,0)
#define COLOR_OBSTACLE_SMALL Scalar(60, 60 ,60)
#define COLOR_OBSTACLE_OLD Scalar(63, 63 ,63);
#define COLOR_FLOOR_TEXT Scalar(0, 190, 190)
#define COLOR_BALL Scalar(0, 255, 255)
#define COLOR_BALL_SMALL Scalar(0, 153, 153)
#define COLOR_CYAN Scalar(255, 255, 0)
#define COLOR_CYAN_SMALL Scalar(139, 139, 0)
#define COLOR_MAGENTA Scalar(255, 0, 255)
#define COLOR_MAGENTA_SMALL Scalar(139, 0, 139)

class configurator {
private:
	std::string configWindow, colorWindowLine, colorWindowFloor, colorWindowBall, colorWindowPossession,
			colorWindowObstacle, colorWindowCyan, colorWindowMagenta, cameraWindow;

	cv::Point centerPoint;
	int centerPixel, viewPixels, cameraRadiusObstacleMin, cameraRadiusObstacleMax, cameraRadiusBallMax,
			cameraRadiusLineMax;
	bool remote; // used for remote viewer only
	positionStInt manual;
	bool manualMode;
	bool calibrateMode;
	bool guiEnabled;
	solverSt solver;
	double tmpRos;
	int goalPostIn, goalPostOut;
	multicastSt multicast;
	goodEnoughSt goodEnough;
	double scoreThresHold;

	colStruct ball, lineSt, obstacle, floorSt, cyan, magenta; // the color window ranges for the 4 targets
	possessionStruct possession;
	cameraSt camera; // containing camera configuration used by camCtrl
	cameraControl camCtrl; // control camera e.g. brightness, hue, exposure
	floorSizeSt floorSize;

	int xOpticalBias, yOpticalBias, zLin;
	exportOffsetIntSt exportOffsetInt;
	exportOffsetDblSt exportOffsetDbl;
	int erodeSlider, dilateSlider;
	deWarpSt deWarp;
	int robot;

	int blurPixels;

	void showTrackbars(int floorWidth, int floorHeight);
	void drawLineBGR();
	void drawFloorBGR();
	void drawBallBGR();
	void drawObstacleBGR();
	void drawCyanBGR();
	void drawMagentaBGR();

public:
	configurator(int robot);
	void init(int floorWidth, int floorHeight, bool guiEnabled);
	void update();

	/* Generic setters and getters */
	floorSizeSt getFloorSize() {
		return floorSize;
	}
	void setFloorSize(floorSizeSt value) {
		floorSize = value;
	}
	int getRobot() {
		return robot;
	}
	bool getRemote() {
		return remote;
	}
	void setRemote(bool value) {
		remote = value;
	}
	int getVideoDevice() {
		return camCtrl.getVideoDevice();
	}
	cameraSt getCamera() {
		return camera;
	}
	void setCamera(cameraSt value) {
		camera = value;
	}
	cv::Point getCenterPoint() {
		return centerPoint;
	}
	int getCenterPixel() {
		return centerPixel;
	}
	int getViewPixels() {
		return viewPixels;
	}
	int getCameraRadiusObstacleMin() {
		return cameraRadiusObstacleMin;
	}
	int getCameraRadiusBallMax() {
		return cameraRadiusBallMax;
	}
	int getCameraRadiusLineMax() {
		return cameraRadiusLineMax;
	}
	int getCameraRadiusObstacleMax() {
		return cameraRadiusObstacleMax;
	}
	bool getGuiEnabled() {
		return guiEnabled;
	}
	int getXOpticalBias() {
		return xOpticalBias;
	}
	void setXOpticalBias(int value) {
		tmpRos = value;
	}
	void setXMetalBias(int value) {
		tmpRos = value;
	}
	int getYOpticalBias() {
		return yOpticalBias;
	}
	void setYOpticalBias(int value) {
		tmpRos = value;
	}
	void setYMetalBias(int value) {
		tmpRos = value;
	}
	exportOffsetDblSt getExportOffset() {
		exportOffsetDblSt exportOffsetDbl;
		exportOffsetDbl.x = (exportOffsetInt.x - 100) / 10.0; // the slider type is positive int, convert to range of -10.0 to + 10.0 in steps of 0.1
		exportOffsetDbl.y = (exportOffsetInt.y - 100) / 10.0;
		exportOffsetDbl.rz = (exportOffsetInt.rz - 100) / 10.0;
		exportOffsetDbl.rzBall = (exportOffsetInt.rzBall - 100) / 10.0;
		exportOffsetDbl.rzObstacle = (exportOffsetInt.rzObstacle - 100) / 10.0;
		return exportOffsetDbl;
	}
	void setXStepSolver(int value) {
		tmpRos = value;
	}
	void setYStepSolver(int value) {
		tmpRos = value;
	}
	void setRzStepSolver(int value) {
		tmpRos = value;
	}
	void setMaxStepsSolver(int value) {
		tmpRos = value;
	}
	int getGoalPostIn() {
		return goalPostIn;
	}
	int getGoalPostOut() {
		return goalPostOut;
	}
	multicastSt getMulticast() {
		return multicast;
	}
	goodEnoughSt getGoodEnough() {
		return goodEnough;
	}
	double getScoreThresHold() {
		return scoreThresHold;
	}
	positionStDbl getManual() {
		positionStDbl pos;
		pos.x = (double)manual.x;
		pos.y = (double)manual.y;
		pos.rz = (double)manual.rz;
		return pos;
	}
	void setManualMode(bool newValue) {
		manualMode = newValue;
	}
	bool getManualMode() {
		return manualMode;
	}
	void setCalibrateMode(bool newValue) {
		calibrateMode = newValue;
	}
	bool getCalibrateMode() {
		return calibrateMode;
	}
	int getBlurPixels() {
		if( calibrateMode ) {
			return 2;
		} else {
			return blurPixels;
		}
	}
	void setBlurPixels(int value) {
		tmpRos = value;
	}

	void setLineHueCentre(int value) {
		tmpRos = value;
	}
	void setLineHueDelta(int value) {
		tmpRos = value;
	}
	void setLineSatMin(int value) {
		tmpRos = value;
	}
	void setLineSatMax(int value) {
		tmpRos = value;
	}
	void setLineSatMaxDebug(int value) {
		lineSt.sat.max = value;
		updateLineTrackbar(0);
	}
	void setLineValMin(int value) {
		tmpRos = value;
	}
	void setLineCalValMin(int value) {
		tmpRos = value;
	}
	void setLineValMax(int value) {
		tmpRos = value;
	}
	void setCameraRadiusMin(int value) {
		tmpRos = value;
	}
	void setCameraRadiusMax(int value) {
		tmpRos = value;
	}
	void setDewarpPixel(double pix1, double pix2) {
		tmpRos = pix1;
		deWarp.pixelPow2 = pix2;
	}
	void setDewarpTan(double inside, double outside) {
		tmpRos = inside;
		deWarp.outSideTan = outside;
	}

	void setBallHueCentre(int value) {
		tmpRos = value;
	}
	void setBallHueDelta(int value) {
		tmpRos = value;
	}
	void setBallSatMin(int value) {
		tmpRos = value;
	}
	void setBallSatMax(int value) {
		tmpRos = value;
	}
	void setBallValMin(int value) {
		tmpRos = value;
	}
	void setBallValMax(int value) {
		tmpRos = value;
	}
	void setBallErode(int value) {
		tmpRos = value;
	}
	void setBallDilate(int value) {
		tmpRos = value;
	}

	void setObstacleHueCentre(int value) {
		tmpRos = value;
	}
	void setObstacleHueDelta(int value) {
		tmpRos = value;
	}
	void setObstacleSatMin(int value) {
		tmpRos = value;
	}
	void setObstacleSatMax(int value) {
		tmpRos = value;
	}
	void setObstacleValMin(int value) {
		tmpRos = value;
	}
	void setObstacleValMax(int value) {
		tmpRos = value;
	}
	void setObstacleErode(int value) {
		tmpRos = value;
	}
	void setObstacleDilate(int value) {
		tmpRos = value;
	}

	double getCameraLatencyOffset() {
		return ((double)camera.latencyOffset / 1000.0);
	}

	solverStDbl getSolver() {
		solverStDbl retVal;
		retVal.xStep = (double)solver.xStep;
		retVal.yStep = (double)solver.yStep;
		retVal.rzStep = (double)solver.rzStep;
		retVal.maxCount = solver.maxCount;
		retVal.score = (double)solver.score / 100.0f;
		return retVal;
	}

	// color information for each type
	colStruct getLine() {
		return lineSt;
	}
	colStruct getFloor() {
		return floorSt;
	}
	colStruct getBall(size_t type);
	possessionStruct getPossession() {
		return possession;
	}
	colStruct getObstacle() {
		return obstacle;
	}

	// function to convert pixels from the distorted round viewer to rectangular soccer floor
	double calcDeWarp(double pixel);

	// Below are internal functions but need to be public to for the static functions
	// that are required for the track bars.
	void updateLineTrackbar(int);
	void updateFloorTrackbar(int);
	void updateBallTrackbar(int);
	void updatePossessionTrackbar(int);
	void updateCyanTrackbar(int);
	void updateMagentaTrackbar(int);
	void updateLineErode(int newValue) {
		lineSt.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateLineDilate(int newValue) {
		lineSt.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateBallErode(int newValue) {
		ball.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateBallDilate(int newValue) {
		ball.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateObstacleTrackbar(int);
	void updateObstacleErode(int newValue) {
		obstacle.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateObstacleDilate(int newValue) {
		obstacle.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateCyanErode(int newValue) {
		cyan.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateCyanDilate(int newValue) {
		cyan.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateMagentaErode(int newValue) {
		magenta.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}
	void updateMagentaDilate(int newValue) {
		magenta.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
				cv::Point(newValue, newValue));
	}

	static void cbUpdateLineTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateLineTrackbar(newValue);
	}

	static void cbUpdateFloorTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateFloorTrackbar(newValue);
	}

	static void cbUpdateBallTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateBallTrackbar(newValue);
	}

	static void cbUpdatePossessionTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updatePossessionTrackbar(newValue);
	}

	static void cbUpdateCyanTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateCyanTrackbar(newValue);
	}

	static void cbUpdateMagentaTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateMagentaTrackbar(newValue);
	}

	static void cbUpdateLineErodeTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateLineErode(newValue);
	}

	static void cbUpdateLineDilateTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateLineDilate(newValue);
	}

	static void cbUpdateBallErodeTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateBallErode(newValue);
	}

	static void cbUpdateBallDilateTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateBallDilate(newValue);
	}

	static void cbUpdateObstacleTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateObstacleTrackbar(newValue);
	}

	static void cbUpdateObstacleErodeTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateObstacleErode(newValue);
	}

	static void cbUpdateObstacleDilateTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateObstacleDilate(newValue);
	}

	static void cbUpdateCyanErodeTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateCyanErode(newValue);
	}

	static void cbUpdateCyanDilateTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateCyanDilate(newValue);
	}

	static void cbUpdateMagentaErodeTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateMagentaErode(newValue);
	}

	static void cbUpdateMagentaDilateTrackbar(int newValue, void *object) {
		configurator *conf = (configurator*)object;
		conf->updateMagentaDilate(newValue);
	}
};

#endif
