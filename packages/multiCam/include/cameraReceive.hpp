// Copyright 2018-2020 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERA_RECEIVE_HPP
#define CAMERA_RECEIVE_HPP

#include "camPacket.hpp"
#include "configurator.hpp"

#include <arpa/inet.h>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define SEND_IMAGE_WIDTH (32*25) // 800
#define SEND_IMAGE_HEIGHT (32*19) // 608

enum camIdList {
	invalid = 0,
	cam0LinePoint = 2,
	cam0Image0 = 3,
	cam0Image1,
	cam0Image2,
	cam0Image3,
	cam1LinePoint,
	cam2LinePoint,
	cam3LinePoint,
	cam1Image,
	cam2Image,
	cam3Image,
	cam0BallPoint = 30,
	cam1BallPoint,
	cam2BallPoint,
	cam3BallPoint,
	cam0ObstaclePoint = 40,
	cam1ObstaclePoint,
	cam2ObstaclePoint,
	cam3ObstaclePoint
};

class cameraReceive {
private:
	configurator *conf;

	std::condition_variable ballDetCondVar, ballFarDetCondVar, obstacleDetCondVar; // used by the ballDetection thread to wait until data available from the receiver

	int fd;
	camPacketT rxPacket;
	uint8_t rxPacketCntExpected[4];
	bool rxPacketCntFirstCheck[4];
	std::vector<linePointSt> linePointListLongAxis[4], linePointListShortAxis[4], floorPointList[4], ballPointList[4],
			ballFarPointList[4], obstaclePointList[4];
	std::mutex statisticsExportMutex, linePointListLongAxisExportMutex, linePointListShortAxisExportMutex,
			ballPointListExportMutex, ballFarPointListExportMutex;
	std::mutex floorPointListExportMutex, obstaclePointListExportMutex, cameraFrameExportMutex;
	bool waitForData;
	cv::Mat cameraFrame[4];
	double receiveTime[4], receiveTimeDelta[4], ballReceiveTime[4];
	size_t deltaTimePrintCounter;
	uint32_t anaFrameCounter[4];
	uint16_t anaApplUptime[4];
	uint8_t camValAverage[4];

	uint8_t cameraFrameRecv[4][3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT];

public:
	cameraReceive(configurator *conf);
	void block();
	void receive(); // blocking, but runs in separate thread
	size_t getLinePointsLongAxisAmount(size_t camIndex);
	size_t getLinePointsShortAxisAmount(size_t camIndex);
	std::vector<linePointSt> getLinePointsLongAxis(size_t camIndex);
	std::vector<linePointSt> getLinePointsShortAxis(size_t camIndex);
	std::vector<linePointSt> getAndClearBallPointsWait(size_t camIndex);
	std::vector<linePointSt> getAndClearBallFarPointsWait(size_t camIndex);

	std::vector<linePointSt> getFloorPoints(size_t camIndex);

	std::vector<linePointSt> getObstaclePointsWait(size_t camIndex);

	cv::Mat getCameraFrame(size_t ii);

	double getRecvDeltaTime(size_t camIndex);

	uint32_t getAnaFrameCounter(size_t camIndex);
	uint16_t getAnaApplUptime(size_t camIndex);
	uint8_t getCamValAverage(size_t camIndex);
};

#endif // CAMERA_RECEIVE_HPP
