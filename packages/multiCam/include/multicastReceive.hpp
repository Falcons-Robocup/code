// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAST_RECEIVE
#define MULTICAST_RECEIVE

#include "configurator.hpp"
#include "determinePosition.hpp"
#include "multicastCommon.hpp"

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
	uint16_t x;
	uint16_t y;
} xyPointSt;

typedef struct {
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
	double localTime; // 64 bits local time in micro seconds since 1 January 1970
	char remoteTimeString[32]; // remote time converted in printable format %H:%M:%S.miliseconds
} statsSt;

typedef struct {
	uint8_t packetCnt;
	int state;
	std::vector<xyPointSt> list;
} linePointListSt;

enum stateMode { invalid=0, updated, old };

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
	float angle;
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

class multicastReceive {
private:
	int fd;
	std::vector<size_t> packetErrorCnt;
	std::vector<uint8_t> packetCnt;
	uint16_t packetSize;
	struct sockaddr_in toAddr;
	packetT recvBuf;
	std::vector<statsSt> stats;
	std::vector<linePointListSt> linePointList;
	std::vector<goodEnoughLocSt> goodEnoughLoc;
	std::vector< std::vector<positionStDbl> > posRosHistory; // moving average filter ros value
	std::vector<positionStDbl> posRosAverage;
	std::vector<locListSt> locList;
	std::vector<ballObstListSt> ballList;
	std::vector<ballObstListSt> cyanList;
	std::vector<ballObstListSt> magentaList;
	std::vector<ballObstListSt> obstacleList;

	int nbytes;
	struct ip_mreq mreq;

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
	std::mutex locListExportMutex, ballListExportMutex, cyanListExportMutex, magentaListExportMutex, obstacleListExportMutex;

	void decodeStats( size_t index );
	void decodeLinePoints( size_t index );
	void decodeGoodEnoughLoc( size_t index );
	void decodeLocalization( size_t index );
	void decodeBallList( size_t index );
	void decodeCyanList( size_t index );
	void decodeMagentaList( size_t index );
	void decodeObstacleList( size_t index );

public:
	multicastReceive( );
	virtual ~multicastReceive();

	void receive( );
	size_t getPacketErrorCnt( size_t index );
	size_t getPossessionThreshold( ) { return possessionThreshold; }
	void packetIndexPauseToggle( ) { packetIndexPause = ! packetIndexPause; }
	void packetIndexAdd( int value );
	statsSt getStats( size_t index );
	linePointListSt getLinePointList( size_t index );
	goodEnoughLocSt getGoodEnoughLoc( size_t index );
	positionStDbl getGoodEnoughLocRosAverage( size_t index );
	locListSt getLocList( size_t index );
	ballObstListSt getBallList( size_t index, size_t type );
	ballObstListSt getObstacleList( size_t index );
};

#endif
