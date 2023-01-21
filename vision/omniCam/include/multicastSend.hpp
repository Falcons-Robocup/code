// Copyright 2017-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAST_SEND

#define MULTICAST_SEND

#include <arpa/inet.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ballDetection.hpp"
#include "ballPossession.hpp"
#include "configurator.hpp"
#include "determinePosition.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "multicastCommon.hpp"
#include "obstacleDetection.hpp"
#include "preprocessor.hpp"
#include "robotFloor.hpp"

class multicastSend {
private:
	int fd;
	uint8_t packetCnt;
	uint16_t packetSize;
	struct sockaddr_in toAddr;
	packetT sendBuf;

	// pointers for access to other classes
	ballDetection *ballDet;
	ballPossession *ballPos;
	configurator *conf;
	ballDetection *cyanDet;
	determinePosition *detPos;
	linePointDetection *linePoint;
	localization *loc;
	ballDetection *magentaDet;
	obstacleDetection *obstDet;
	preprocessor *prep;
	robotFloor *rFloor;

public:

	multicastSend(ballDetection *ballDet, ballPossession *ballPos, configurator *conf, ballDetection *cyanDet,
			determinePosition *detPos, linePointDetection *linePoint, localization *loc, ballDetection *magentaDet,
			obstacleDetection *obstDet, preprocessor *prep, robotFloor *rFloor);
	void send();
	void printNetworkInterface(char *buff, int flags);
	void stats();
	void locList();
	void goodEnoughLoc();
	void floorLinePoints();
	void ballList(size_t type);
	void obstaclelList();
};

#endif
