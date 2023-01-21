// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef OBSTACLEDETECTION_HPP
#define OBSTACLEDETECTION_HPP

#include <mutex>
#include <opencv2/opencv.hpp>

#include "configurator.hpp"
#include "preprocessor.hpp"

typedef struct {
	int xLeft;
	int xRight;
	int yTop;
	int yBottom;
	int size;
	double radiusTan;
	double radiusLin;
	double angle;
} obstacleSt;

class obstacleDetection {

private:
	// pointers for access to other classes
	preprocessor *prep;
	configurator *conf;

	std::vector<obstacleSt> positions, positionsExport;
	cv::Mat blackFloor, obstacleFrame, obstacleErodeFrame, obstacleDilateFrame, obstacleMask;

	int cameraRadiusObstacleMinPrevious;
	int cameraRadiusObstacleMaxPrevious;
	cv::Point centerPointPrevios;
	std::mutex exportMutex;
	bool busy;

public:
	obstacleDetection(configurator *conf, preprocessor *prep);
	void update(int robotId);
	std::vector<obstacleSt> getPositions();
	std::vector<obstacleSt> getPositionsExport();

	cv::Mat getObstacle();
	cv::Mat getObstacleErode();
	cv::Mat getObstacleDilate();

	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}

};
#endif
