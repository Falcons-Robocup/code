// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef ZYNQ_CONTROL_HPP
#define ZYNQ_CONTROL_HPP

#include <cstdint>

#include "tcp_socket.hpp"

class zynqControl {
private:
	tcp_socket *sock[4];

	std::string lineConfig, ballConfig, obstacleConfig, floorConfig, cameraConfig, sizeConfig;
	int lineValMinPrev, lineSatMaxPrev;
	int lineTransferPixelsMaxPrev, lineFloorWindowSizePrev, lineFloorPixelsMinPrev, lineWindowSizePrev,
			linePixelsMinPrev;
	int ballValMinPrev, ballSatMinPrev, ballHueMinPrev, ballHueMaxPrev;
	int ballWindowSizePrev, ballPixelsMinPrev, ballFalsePixelsMaxPrev;
	int floorValMinPrev, floorSatMinPrev, floorHueMinPrev, floorHueMaxPrev;
	int obstacleValMaxPrev, obstacleSatMaxPrev;
	int obstacleFloorWindowSizePrev, obstacleLineWindowSizePrev, obstacleBallWindowSizePrev, obstacleWindowSizePrev;
	int obstacleTransferPixelsMaxPrev, obstacleFloorPixelsMinPrev, obstacleLinePixelsMinPrev, obstacleBallPixelsMinPrev;
	int obstaclePixelsMinPrev;


	int redPrev, greenPrev, bluePrev;
	int testPatternPrev, analogGainPrev, shutterPrev;
	int linesPrev, pixelsPrev, xStartPrev, xEndPrev, xSizePrev, yStartPrev, yEndPrev, ySizePrev;

	int cameraReset, EXCK_FREQ_PREV, VTPXCK_DIV_PREV, VTSYCK_DIV_PREV, PREPLLCK_VT_DIV_PREV, PREPLLCK_OP_DIV_PREV;
	int PLL_VT_MPY_PREV, OPPXCK_DIV_PREV, OPSYCK_DIV_PREV, PLL_OP_MPY_PREV;

	void send_message(uint8_t id, uint32_t value);
	void send_message(uint8_t id);

public:
	zynqControl();
	void update( bool interfaceLocal );
};

#endif
