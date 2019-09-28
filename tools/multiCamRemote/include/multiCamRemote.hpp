// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTI_CAM_REMOTE_HPP
#define MULTI_CAM_REMOTE_HPP

#include "configurator.hpp"
#include "multicastReceive.hpp"
#include "robotFloor.hpp"
#include <thread>

class multiCamRemote {
private:
	configurator *conf;
	multicastReceive *multRecv;
	robotFloor *rFloor;

	int width, height;
	int xCenter, yCenter;
	double remoteRatio; // used to convert robot pixels to multiCamRemote viewer pixels

	cv::Mat grassFrame, allFrame;

	// create the frame for all 6 robots
	std::vector<cv::Mat> robots; // latest and greatest viewer image
	std::vector<bool> flip;
	bool robotAlive[MAX_ROBOTS];

	std::thread receiveThread;

	void checkIfRobotStillAlive(size_t index);
	void drawFloorLinePoints(size_t index);
	void drawFloorRobot(size_t index);
	void drawFloorBalls(size_t index, cv::Scalar color, cv::Scalar colorDark, size_t type);
	void drawFloorObstacles(size_t index);
	void floorPrintText(size_t index, cv::Scalar color);
	void printBallPosition(size_t index, size_t type, size_t leftIndent, int line, cv::Scalar color);

public:
	multiCamRemote();
	void update();
	void packetIndexPauseToggle() {
		multRecv->packetIndexPauseToggle();
	}
	void packetIndexAdd(int value);
	void startReceiveThread();
	void doFlip(size_t index) {
		flip[index] = !flip[index];
	}
};

#endif
