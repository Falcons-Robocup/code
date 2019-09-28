// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAST_RECEIVE
#define MULTICAST_RECEIVE

#include "configurator.hpp"
#include "multicastCommon.hpp"

// should be at least 6
#define MAX_ROBOTS 8

#include "opencv2/highgui/highgui.hpp"
// TODO: replace with something that only has std::vector

#include <arpa/inet.h>
#include <mutex>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef struct {
	uint16_t sysApplUptime; // system application uptime in seconds (wrap around at 18.2 hours)
	int8_t cmosTemp; // imx219 camera sensor temperature in Celsius
	uint8_t cpuLoad; // warning: fixed point, divide by 32.0 to get correct value, note: there are 4 physical cores on the raspi 3b+
	uint8_t cpuTemp; // A53 CPU temperature in Celsius
	uint16_t cpuUptime; // CPU uptime in seconds (wrap around at 18.2 hours)
	uint8_t gpuTemp; // GPU temperature in Celsius
	uint8_t cpuStatus; // not decoded: underVoltage, frequencyCapped, throttling
	uint8_t camValAverage; // average camera value, used to determine if one of the cameras is not correctly configured (too dark)
	uint8_t rebootReceivedAge; // age in seconds when the going to reboot message was received from raspi
	uint16_t anaApplUptime; // analyze application uptime in seconds (wrap around at 18.2 hours)
	uint32_t anaFrameCounter; // analyze application frame counter
} raspiStatsSt;

struct statsSt {
	size_t robotIndex; // from 0 to n instead of 1 to m
	uint8_t packetCnt;
	uint8_t ballAmount;
	uint8_t cyanAmount;
	uint8_t magentaAmount;
	uint8_t obstacleAmount;
	uint16_t possessionPixels; // amount of ball pixels seen just before the shooter
	uint16_t linePoints; // total amount of line points found
	float uptime; // uptime since application startup
	float prepFps; // frames per second for balls and obstacles
	float locFps; // frames per second for localization
	uint32_t prepFrames; // amount of frames since application start
	uint32_t locFrames; // amount of localizations performed since application start
	uint64_t remoteTime; // 64 bits remote time in micro seconds since 1 January 1970
	uint64_t localTime; // 64 bits local time in micro seconds since 1 January 1970
	char remoteTimeString[32]; // remote time converted in printable format %H:%M:%S.miliseconds
	raspiStatsSt raspi[4];
	bool operator<(const statsSt& val) const {
		// sorting this struct is performed on received time (localTime) (received later is better)
		return localTime > val.localTime;
	}
};

enum stateMode {
	invalid = 0, updated, old
};

typedef struct {
	uint8_t packetCnt;
	int state;
	positionStDbl pos;
	positionStDbl posRos;
	bool goodEnough;
	double score;
	int lastActive; // last time (in frames) location was found
	int age; // life time in amount of frames
	int amountOnFloor;
	int numberOfTries;
} goodEnoughLocSt;

typedef struct {
	int size;
	float radius;
	float azimuth;
	float elevation;
} ballObstSt;

typedef struct {
	uint8_t packetCnt;
	int state;
	std::vector<ballObstSt> list;
} ballObstListSt;

typedef struct {
	uint8_t packetCnt;
	std::vector<detPosSt> list;
} locListSt;

typedef struct {
	size_t index;
	std::vector<uint8_t> data;
} packetSt;

struct hhMmSsSt {
	uint32_t hours;
	uint32_t minutes;
	uint32_t seconds;
};

class multicastReceive {
private:
	int fd;
	std::vector<size_t> packetErrorCnt;
	std::vector<uint8_t> packetCnt;
	uint16_t packetSize;
	struct sockaddr_in toAddr;
	packetT recvBuf;
	std::vector<statsSt> stats;
	std::vector<ballObstListSt> linePointList;
	std::vector<goodEnoughLocSt> goodEnoughLoc;
	std::vector<std::vector<positionStDbl> > posRosHistory; // moving average filter ros value
	std::vector<positionStDbl> posRosAverage;
	std::vector<locListSt> locList;
	std::vector<ballObstListSt> ballList;
	std::vector<ballObstListSt> ballFarList;
	std::vector<ballObstListSt> cyanList;
	std::vector<ballObstListSt> magentaList;
	std::vector<ballObstListSt> obstacleList;

	int nbytes;

	FILE *writeTxt, *writeBin, *readBin; // handles to recording files
	std::vector<packetSt> packets;
	size_t packetIndex;
	size_t packetIndexPrev;
	bool packetIndexPause;

	int possessionThreshold;

	char ipAddress[INET_ADDRSTRLEN];
	struct sockaddr fromAddr;
	socklen_t fromAddrLen;

	std::mutex statsExportMutex, linePointListExportMutex, goodEnoughLocExportMutex;
	std::mutex locListExportMutex, ballListExportMutex, ballFarListExportMutex, cyanListExportMutex,
			magentaListExportMutex, obstacleListExportMutex;

	void decodeStats(size_t index);
	void decodeLinePoints(size_t index);
	void decodeGoodEnoughLoc(size_t index);
	void decodeLocalization(size_t index);
	void decodeBallList(size_t index);
	void decodeBallFarList(size_t index);
	void decodeCyanList(size_t index);
	void decodeMagentaList(size_t index);
	void decodeObstacleList(size_t index);

public:
	multicastReceive();
	virtual ~multicastReceive();

	void receive();
	size_t getPacketErrorCnt(size_t index);
	size_t getPossessionThreshold() {
		return possessionThreshold;
	}
	void packetIndexPauseToggle() {
		packetIndexPause = !packetIndexPause;
	}
	bool getPacketIndexPause() {
		return packetIndexPause;
	}
	void packetIndexAdd(int value);
	statsSt getStats(size_t index);
	std::vector<statsSt> getStatsSort();
	ballObstListSt getLinePointList(size_t index);
	goodEnoughLocSt getGoodEnoughLoc(size_t index);
	positionStDbl getGoodEnoughLocRosAverage(size_t index);
	locListSt getLocList(size_t index);
	ballObstListSt getBallList(size_t index, size_t type);
	ballObstListSt getBallFarList(size_t index, size_t type);
	ballObstListSt getObstacleList(size_t index);
	hhMmSsSt secondsToHhMmSs(uint16_t input);
};

#endif
