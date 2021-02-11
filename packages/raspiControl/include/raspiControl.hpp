// Copyright 2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef RASPI_CONTROL_HPP
#define RASPI_CONTROL_HPP

#include "camSysReceive.hpp"
#include "camPacket.hpp"

#include <arpa/inet.h>
#include <cstdint>
#include <sys/socket.h>
#include <thread>

class raspiControl {
private:
	struct sockaddr_in toAddr;
	int multFd;

	camPacketT txPacket;
	uint8_t txPacketCnt;

	camSysReceive *camSysRecv;
	std::thread camSysRecvThread;
	double systemControlSendTime;

	// camera clocks (imx219)
	int EXCK_FREQ;
	int VTPXCK_DIV;
	int VTSYCK_DIV;
	int PREPLLCK_VT_DIV;
	int PREPLLCK_OP_DIV;
	int PLL_VT_MPY;
	int OPPXCK_DIV;
	int OPSYCK_DIV;
	int PLL_OP_MPY;

	// camera control and white balance (software)
	int cameraReset;
	int cameraVerbose;
	int testPattern;
	int analogGain;
	int shutter;
	int blackLevel;
	int redMult;
	int blueMult;

	// camera dimensions
	int lines;
	int pixels;
	int pixelsOffset[4];
	int xStart;
	int xEnd;
	int xSize;
	int yStart;
	int yEnd;
	int ySize;
	int imageOrientation;

	// line point detection
	int lineValMin;
	int lineSatMax;
	int lineTransferPixelsMax;
	int lineFloorWindowSize;
	int lineFloorPixelsMin;
	int lineWindowSize;
	int linePixelsMin;

	// ball point detection
	int ballValMin;
	int ballSatMin;
	int ballHueMin;
	int ballHueMax;
	int ballWindowSize;
	int ballPixelsMin;
	int ballFalsePixelsMax;

	// ballFar point detection
	int ballFarValMin;
	int ballFarSatMin;
	int ballFarHueMin;
	int ballFarHueMax;
	int ballFarWindowSize;
	int ballFarPixelsMin;
	int ballFarFalsePixelsMax;

#ifdef WHITE_BLACK_BALL_SEARCH
	int ballWhiteWindowSize;
	int ballWhitePixelsMin;
	int ballBlackWindowSize;
	int ballBlackPixelsMin;
	int ballWhiteBlackFalsePixelsMax;
#endif

	// floor point detection
	int floorValMin;
	int floorSatMin;
	int floorHueMin;
	int floorHueMax;

	// obstacle point detection
	int obstacleValMax;
	int obstacleSatMax;
	int obstacleFloorWindowSize;
	int obstacleFloorPixelsMin;
	int obstacleLineWindowSize;
	int obstacleLinePixelsMin;
	int obstacleBallWindowSize;
	int obstacleBallPixelsMin;
	int obstacleTransferPixelsMax;
	int obstacleWindowSize;
	int obstaclePixelsMin;

	// grabber configuration
	int grabberRedGain;
	int32_t grabberBlueGain;

public:
	raspiControl();
	void multiCastSetup();
	void multiCastClose();
	void update();
	void reboot();
	void poweroff();
	void grabControl();
	void requestVideoStream() {
		analyzeControl();
	}
	void analyzeControl();
	void camControlSetup();
	void grabControlSetup();
	void camControl();
	void systemControl();
	void cam0FrameCntSend(uint32_t cam0FrameCounter);
	void systemControlPixelsOffset();
	void updateSystemControlPixelsOffset(int pixelsOffset[4]);

};

#endif
