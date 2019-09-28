// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h> // for O_RDWR
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <netinet/in.h>
#include <net/if.h> // for if_nametoindex
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h> // for shared memory
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h> // usleep

#include "raspiDefaults.hpp"

// #define LATENCY_TIME_MEASUREMENT
// #define CALCULATION_TIME_MEASUREMENT

// gpio register for raspi 2 and 3
#define GPIO_REGISTER_BASE 0x3F000000 + 0x200000
#define GPIO_SET_OFFSET 0x1C
#define GPIO_CLR_OFFSET 0x28
#define GPIO_PIN_BALL 20
#define GPIO_PIN_LED 16

volatile uint32_t *gpio_set_ptr;
volatile uint32_t *gpio_clear_ptr;

bool keepRunning = true; // for graceful main and thread shutdown
bool continueProcessOneFrame = true; // for testing on x86_64
uint8_t camIndex = 0; // provided through command line
uint16_t imageSendFrames = 0; // on request the remote can ask for a number of frames
bool imageSendBusy = false;
int multReceiveFd = 0;
int multReceiveLoopBackFd = 0;
int multSendFd = 0;
struct sockaddr_in toAddr;
uint32_t frameCounterPrev = 0;
float analyzeApplicationStartTime = 0;
uint16_t analyzeApplicationUptime = 0;
pid_t processId = 0;
uint32_t valueSum[4] = {0, 0, 0, 0}; // 4 threads, total sum of all values fit in 32 bits: 255*800*608 = 0x7582080

#ifdef LATENCY_TIME_MEASUREMENT
uint8_t latencyMeasureWait = 0;
uint64_t ledSetTime = 0xffffffffffffffffL;
uint64_t shmSetTime = 0xffffffffffffffffL;
uint64_t minLedBalDelta = 0xffffffffffffffffL;
uint64_t maxLedBalDelta = 0;
uint64_t avgLedBalDelta = 0;
uint64_t avgLedBalDeltaAmount = 0;
uint64_t avgLedShmDelta = 0;
#endif

#ifdef CALCULATION_TIME_MEASUREMENT
uint64_t ballSetTime = 0xffffffffffffffffL;
uint64_t avgShmBallDelta = 0xffffffffffffffffL;
uint64_t avgShmDoneDelta = 0xffffffffffffffffL;
uint64_t avgShmDoneDeltaAmount = 0;
#endif

#define SHM_SIZE ( 3 * ROI_WIDTH * ROI_HEIGHT ) // 3 bytes per pixel
uint8_t *shmData = 0;
uint8_t *shmDataSend = 0;

typedef enum colorDetectionEnum {
	DET_DEFAULT = 0,
	DET_LINE = 1,
	DET_BALL = 2,
	DET_BALL_FAR = 4,
	DET_OBSTACLE = 8,
	DET_CYAN = 16,
	DET_FLOOR = 32,
	DET_ZZ_LAST = 128
} colorDetectionT;

uint8_t colorDetectionBuffer[ROI_HEIGHT * ROI_WIDTH];
bool raspiColorDetectionBusy = false;

pthread_t raspiColorDetectionThreadId[4];
pthread_t raspiBallDetectionThreadId;
// pthread_t raspiBallFarDetectionThreadId;
pthread_t raspiFloorDetectionThreadId;
pthread_t raspiObstacleDetectionThreadId;
pthread_t raspiLineDetectionLongAxisThreadId;
pthread_t raspiLineDetectionShortAxisThreadId;
pthread_t raspiStatisticsThreadId;
pthread_t raspiConfigThreadId;
pthread_t gpioLedToggleThreadId;

// line point detection
uint16_t lineValMin = LINE_VAL_MIN;
uint16_t lineSatMax = LINE_SAT_MAX;
uint16_t lineTransferPixelsMax = LINE_TRANSFER_PIXELS_MAX;
uint16_t lineFloorWindowSize = LINE_FLOOR_WINDOW_SIZE;
uint16_t lineFloorPixelsMin = LINE_FLOOR_PIXELS_MIN;
uint16_t lineWindowSize = LINE_WINDOW_SIZE;
uint16_t linePixelsMin = LINE_PIXELS_MIN;

// ball point detection
uint16_t ballValMin = BALL_VAL_MIN;
uint16_t ballSatMin = BALL_SAT_MIN;
uint16_t ballHueMin = BALL_HUE_MIN;
uint16_t ballHueMax = BALL_HUE_MAX;
uint16_t ballWindowSize = BALL_WINDOW_SIZE;
uint16_t ballPixelsMin = BALL_PIXELS_MIN;
uint16_t ballFalsePixelsMax = BALL_FALSE_PIXELS_MAX;

// ballFar point detection
uint16_t ballFarValMin = BALL_FAR_VAL_MIN;
uint16_t ballFarSatMin = BALL_FAR_SAT_MIN;
uint16_t ballFarHueMin = BALL_FAR_HUE_MIN;
uint16_t ballFarHueMax = BALL_FAR_HUE_MAX;
uint16_t ballFarWindowSize = BALL_FAR_WINDOW_SIZE;
uint16_t ballFarPixelsMin = BALL_FAR_PIXELS_MIN;
uint16_t ballFarFalsePixelsMax = BALL_FAR_FALSE_PIXELS_MAX;

#ifdef WHITE_BLACK_BALL_SEARCH
uint16_t ballWhiteWindowSize = BALL_WHITE_WINDOW_SIZE;
uint16_t ballWhitePixelsMin = BALL_WHITE_PIXELS_MIN;
uint16_t ballBlackWindowSize = BALL_BLACK_WINDOW_SIZE;
uint16_t ballBlackPixelsMin = BALL_BLACK_PIXELS_MIN;
uint16_t ballWhiteBlackFalsePixelsMax = BALL_WHITE_BLACK_FALSE_PIXELS_MAX;
#endif

// floor point detection
uint16_t floorValMin = FLOOR_VAL_MIN;
uint16_t floorSatMin = FLOOR_SAT_MIN;
uint16_t floorHueMin = FLOOR_HUE_MIN;
uint16_t floorHueMax = FLOOR_HUE_MAX;

// obstacle point detection
// near the robot there might be shades (from e.g. ball) or robot parts, start a bit later for obstacles
#define OBSTACLE_X_START 15
uint16_t obstacleValMax = OBSTACLE_VAL_MAX;
uint16_t obstacleSatMax = OBSTACLE_SAT_MAX;
uint16_t obstacleFloorWindowSize = OBSTACLE_FLOOR_WINDOW_SIZE;
uint16_t obstacleFloorPixelsMin = OBSTACLE_FLOOR_PIXELS_MIN;
uint16_t obstacleLineWindowSize = OBSTACLE_LINE_WINDOW_SIZE;
uint16_t obstacleLinePixelsMin = OBSTACLE_LINE_PIXELS_MIN;
uint16_t obstacleBallWindowSize = OBSTACLE_BALL_WINDOW_SIZE;
uint16_t obstacleBallPixelsMin = OBSTACLE_BALL_PIXELS_MIN;
uint16_t obstacleTransferPixelsMax = OBSTACLE_TRANSFER_PIXELS_MAX;
uint16_t obstacleWindowSize = OBSTACLE_WINDOW_SIZE;
uint16_t obstaclePixelsMin = OBSTACLE_PIXELS_MIN;

// cyan color detection
// TODO: add to configuration
uint8_t cyanValMin = 80;
uint8_t cyanSatMin = 50;
uint8_t cyanHueMin = 90 - 15; // cyan is 180 degrees
uint8_t cyanHueMax = 90 + 15;

typedef struct {
	uint8_t id;
	uint8_t cnt;
	uint16_t size; // packet size including header in bytes, payload size maximal is 65503 instead of 65536, see below
	union {
		// 2^16 = 64 KiB = 65536 bytes, but limited 65503 bytes, see below
		uint8_t u8[1 << 16]; // unsigned payload
		uint16_t u16[1 << 15];
		uint32_t u32[1 << 14];
		uint64_t u64[1 << 13];
		int8_t s8[1 << 16]; // signed payload
		int16_t s16[1 << 15];
		int32_t s32[1 << 14];
		int64_t s64[1 << 13];
		float f32[1 << 14];
		double d64[1 << 13];
	} pl;
}__attribute__((packed)) packetT;

#define CAM_PACKET_HEADER_SIZE 4
#define IPV4_MAX_SIZE 65535 // 2^16 - 1, TODO: why not 65536 ?
#define IPV4_IP_HEADER 20
#define UDP_PACKET_MAX_SIZE (IPV4_MAX_SIZE - IPV4_IP_HEADER) // 65515
#define UDP_HEADER_SIZE 8
#define UDP_PACKET_PAYLOAD_SIZE ( UDP_PACKET_MAX_SIZE - UDP_HEADER_SIZE ) // 65507
#define CAM_PACKET_PAYLOAD_SIZE (UDP_PACKET_PAYLOAD_SIZE - CAM_PACKET_HEADER_SIZE ) // 65503

packetT txPacket;
uint8_t txPacketCnt = 0;
pthread_mutex_t txMutex;

void multiCastSendSetup() {
	// create a normal UDP socket
	if ((multSendFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cam %u analyze cannot create UDP socket for sending!, message %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address for data
	memset((char *) &toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(44444); // data port

	printf("INFO      : cam %u analyze mulitcast send      IP group address %s port %u\n", camIndex,
			inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));

	// initialize mutex used when sending multicast data
	if (pthread_mutex_init(&txMutex, NULL) != 0) {
		printf("ERROR     : cam %u analyze multicast send cannot initialize txMutex, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// TODO: add packetCntSyncSend() to synchronize packet counter on x86_64 CPU;
}

void multiCastSendClose() {
	close(multSendFd);
	pthread_mutex_destroy(&txMutex);
}

inline void sendThePacket(char *printString) {
	ssize_t sendAmount = sendto(multSendFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (sendAmount < 0) {
		printf("ERROR     : cam %u analyze cannot send %s packet, message %s\n", camIndex, printString,
				strerror(errno));
		printf("WARNING   : cam %u analyze exit now\n", camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (sendAmount != txPacket.size) {
		printf("WARNING   : cam %u analyze only %zd bytes instead of %u for %s packet\n", camIndex, sendAmount,
				txPacket.size, printString);
		fflush(stdout);
	}
}

void multiCastReceiveSetup() {
	// create a normal UDP socket
	if ((multReceiveFd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cam %u analyze cannot create UDP socket for receiving!, message %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port (required when multiple simulation instances are running on x86)
	int reuse = 1;
	if (setsockopt(multReceiveFd, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse)) < 0) {
		printf("ERROR     : cam %u analyze cannot configure port for multiple UDP sockets!, message %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// fill in the local address structure
	struct sockaddr_in localAddr;
	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	// on the raspi use the IP address of the Ethernet interface
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY); // accept any incoming messages
	localAddr.sin_port = htons(22222); // configuration port
	// bind to the local address / port
	if (bind(multReceiveFd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		printf("ERROR     : cam %u analyze cannot bind to UDP socket for receiving, %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset(&mreq, 0, sizeof(mreq)); // set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
		printf("ERROR     : cam %u analyze cannot join the multicast group for receiving, %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	printf("INFO      : cam %u analyze multicast receive   IP group address %s port %u\n", camIndex,
			inet_ntoa(mreq.imr_multiaddr), ntohs(localAddr.sin_port));
}

void multiCastReceiveClose() {
	close(multReceiveFd);
}

// multiCastReceiveLoopBackSetup used to provide data from this application the receive configuration port of this same application (loopback)
void multiCastReceiveLoopBackSetup() {
	// create a normal UDP socket
	if ((multReceiveLoopBackFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf(
				"ERROR     : cam %u analyze cannot create UDP socket for sending to local configuration port!, message %s\n",
				camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address for control
	memset((char *) &toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(22222); // configuration port

	printf("INFO      : cam %u analyze mulitcast loop back IP group address %s port %u\n", camIndex,
			inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));
}

void multiCastReceiveLoopBackClose() {
	close(multReceiveLoopBackFd);
}

void multiCastReceiveLoopBackStop() {
	packetT lbTxPacket;
	lbTxPacket.pl.u32[0] = 0xdead0033; // magic for stop
	lbTxPacket.id = 100;
	lbTxPacket.cnt = 0; // packet counter is not used on receive because data can come from unrelated sources
	lbTxPacket.size = CAM_PACKET_HEADER_SIZE + 4;

	// sendBuf.size is the number of bytes to send
	ssize_t sendAmount = sendto(multReceiveLoopBackFd, &lbTxPacket, lbTxPacket.size, 0, (struct sockaddr *) &toAddr,
			sizeof(toAddr));
	if (sendAmount < 0) {
		printf("ERROR     : cam %u analyze cannot send application exit loop back message, %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (sendAmount != lbTxPacket.size) {
		printf("WARNING   : cam %u analyze only send %zd bytes to loop back instead of %d bytes\n", camIndex,
				sendAmount, lbTxPacket.size);
	}
}

#ifdef NONO
// replaced by the systemRaspi jpg image
// send the image to the CPU box
// every call it only sends one part to reduce the network load (64KiB)
uint8_t imagePart = 0;
inline void imageSend() {
	// the image does not fit in one packet, send in multiple packets, use packet id to inform the
	// the receiver which part of the image it received
	ssize_t remainder = CAM_PACKET_PAYLOAD_SIZE;
	size_t bufferIndex = imagePart * CAM_PACKET_PAYLOAD_SIZE;
	size_t totalSuggested = bufferIndex + CAM_PACKET_PAYLOAD_SIZE;// previous index + one maximal packet payload size
	bool imagePartLast = false;
	if (totalSuggested >= SHM_SIZE) {
		// last part of the image which, which very likely will be smaller then the maximal payload size
		remainder = SHM_SIZE - bufferIndex;// calculate the remainder of the image that has to be send
		imagePartLast = true;// after sending, reset the counters and inform process it can provide a new image
	}

	if (remainder <= 0) {
		printf("ERROR     : cam %u analyze image send calculation error %zd %zd %zd %u\n", camIndex, bufferIndex,
				totalSuggested, remainder, SHM_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (remainder > CAM_PACKET_PAYLOAD_SIZE) {
		printf("ERROR     : cam %u analyze image send calculation error, so to much %zd %zd, %zu %u\n", camIndex,
				bufferIndex, totalSuggested, remainder, SHM_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	// copy a part of the image to the tx packet payload
	memcpy(&txPacket.pl, &shmDataSend[bufferIndex], remainder);
	txPacket.id = camIndex * 64 + 32 + imagePart;// bits [7:6] to identify camera, bit[5] indicate image, bits [4:0] which part (0 to 31)
	txPacket.cnt++;// wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + remainder;

	sendThePacket((char *) "image send");
	pthread_mutex_unlock(&txMutex);

	//printf(
	//		"INFO      : cam %u analyze image part %2u buffer index %7zu send %zd bytes sum %7zu bytes of total %zu bytes tx buffer size %u %zd\n",
	//		camIndex, imagePart, bufferIndex, sendAmount, totalSuggested, SHM_SIZE, txPacket.size, remainder);
	if (imagePartLast) {
		// printf("INFO      : cam %u analyze image send done\n", camIndex);
		imagePart = 0;
		imageSendBusy = false;// handshake to double buffering, imageSend will be called again when new frame is available
		imageSendFrames--;// reduce the requested frames from the x86_64 by 1
	} else {
		imagePart++;
	}
} // imageSend

inline void imageSendConditional() {
	// imageSendFrames set by x86_64 to request image data, and decreased for every frame
	if (imageSendFrames > 0) {
		// the camera image is transmitted in sections that fit in one UDP packet
		// this function is called 40 times per second
		// the maximal UDP packet size is 65535 byte => 2.6 Mbyte/sec => 20.9 Mbit/sec
		if (imageSendBusy) {
			// only send an image if the sendBuffer has been updated
			imageSend();
		}
	}
}
#endif

typedef struct {
	uint16_t xBegin;
	uint16_t xEnd;
	uint16_t yBegin;
	uint16_t yEnd;
} linePointSt;

#define POINTS_MAX_AMOUNT (CAM_PACKET_PAYLOAD_SIZE / (2*4) ) // 65503 / 8 = 8187
linePointSt foundPointsLongAxis[POINTS_MAX_AMOUNT]; // in normal cases about 200 line points will be stored
linePointSt foundPointsShortAxis[POINTS_MAX_AMOUNT]; // in normal cases about 200 line points will be stored
size_t linePointsIndexLongAxis = 0; // amount of found line points found on long camera axis
size_t linePointsIndexShortAxis = 0; // amount of found line points found on short camera axis

// send statistics (minimal latency to capture time difference between the 4 camera's)
inline void statisticsSend() {

// running the following command takes to much time, instead read directly from /proc
#ifdef NONO
	FILE *fp = NULL;
	char command[64];
	if (processId != 0) {
		sprintf(command, "ps -o etimes= -p %d", processId);
		fp = popen(command, "r");
		if (fp == NULL) {
			printf("ERROR     : cam %u analyze application cannot execute %s, message: %s\n", camIndex, command, strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
		int tmpInt = -1;
		fscanf(fp, "%d", &tmpInt); // analyze application uptime in seconds
		if ((tmpInt >= 0) && (tmpInt <= 0xffff)) {
			analyzeApplicationUptime = (uint16_t) tmpInt;
		} else {
			analyzeApplicationUptime = 0xffff;
		}
		pclose(fp);
	} else {
		analyzeApplicationUptime = 0xffff;
	}
#endif

	FILE *fp = fopen("/proc/uptime", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u analyze application cannot open /proc/uptime, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	float cpuUptime = -1.0;
	// CPU uptime in seconds (with fraction)
	if (fscanf(fp, "%f ...", &cpuUptime) <= 0) {
		printf("ERROR     : cam %u analyze cannot extract the CPU uptime from /proc/uptime, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	fclose(fp);

	float uptime = cpuUptime - analyzeApplicationStartTime;
	if ((uptime >= 0.0) && (uptime <= 65535.0)) {
		analyzeApplicationUptime = (uint16_t) uptime;
	} else {
		analyzeApplicationUptime = 0xffff;
	}

	pthread_mutex_lock(&txMutex);
	txPacket.pl.u32[0 / 4] = frameCounterPrev;
	txPacket.pl.u16[4 / 2] = analyzeApplicationUptime; // application uptime in seconds
	txPacket.pl.u8[6 / 1] = ( valueSum[0] + valueSum[1] + valueSum[2] + valueSum[3] ) / (ROI_WIDTH*ROI_HEIGHT); // used to determine if one of the cameras was incorrectly configured (e.g. to dark)

	txPacket.id = camIndex * 64 + 1;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 7;

	sendThePacket((char *) "statics");
	pthread_mutex_unlock(&txMutex);
}

// send the floor long axis line points
inline void linePointsSendLongAxis() {
	// one line point is : xBegin, xEnd, yBegin and yEnd (which are 2 bytes each)
	size_t payloadSize = linePointsIndexLongAxis * sizeof(linePointSt);
	if (payloadSize > CAM_PACKET_PAYLOAD_SIZE) {
		printf(
				"ERROR     : cam %u analyze send %zu long axis line points of %zu bytes do not fit in tx packet payload of %u bytes\n",
				camIndex, linePointsIndexLongAxis, sizeof(linePointSt), CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for (size_t ii = 0; ii < linePointsIndexLongAxis; ii++) {
		txPacket.pl.u16[4 * ii] = foundPointsLongAxis[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = foundPointsLongAxis[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = foundPointsLongAxis[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = foundPointsLongAxis[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 2;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char *) "long axis line points");
	pthread_mutex_unlock(&txMutex);
}

// send the floor short axis line points
inline void linePointsSendShortAxis() {
	// one line point is : xBegin, xEnd, yBegin and yEnd (which are 2 bytes each)
	size_t payloadSize = linePointsIndexShortAxis * sizeof(linePointSt);
	if (payloadSize > CAM_PACKET_PAYLOAD_SIZE) {
		printf(
				"ERROR     : cam %u analyze send %zu short axis line points of %zu bytes do not fit in tx packet payload of %u bytes\n",
				camIndex, linePointsIndexShortAxis, sizeof(linePointSt), CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for (size_t ii = 0; ii < linePointsIndexShortAxis; ii++) {
		txPacket.pl.u16[4 * ii] = foundPointsShortAxis[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = foundPointsShortAxis[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = foundPointsShortAxis[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = foundPointsShortAxis[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 6;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char *) "short axis line points");
	pthread_mutex_unlock(&txMutex);
}

linePointSt floorPoints[POINTS_MAX_AMOUNT]; // 8187, in normal cases about 1000 line points will be stored
size_t floorPointsIndex = 0; // amount of found floor points

// send the floor points
inline void floorPointsSend() {
	// one floor point is : xBegin, xEnd, yBegin and Yend (which are 2 bytes each)
	size_t payloadSize = floorPointsIndex * sizeof(linePointSt);
	if (payloadSize > CAM_PACKET_PAYLOAD_SIZE) {
		printf(
				"ERROR     : cam %u analyze send %zu floor points of %zu bytes, total %zu bytes, which does not fit in tx buffer of %u bytes\n",
				camIndex, floorPointsIndex, sizeof(linePointSt), payloadSize, CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for (size_t ii = 0; ii < floorPointsIndex; ii++) {
		txPacket.pl.u16[4 * ii] = floorPoints[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = floorPoints[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = floorPoints[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = floorPoints[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 5;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char *) "floor points");
	pthread_mutex_unlock(&txMutex);
	// printf("INFO      : cam %u send floor points %zu\n", camIndex, floorPointsIndex);
}

linePointSt ballPoints[POINTS_MAX_AMOUNT]; // 8187, in normal cases about 50 line points will be stored
size_t ballPointsIndex = 0; // amount of found ball points

// send the ball points
inline void ballPointsSend() {
	// one ball point is : xBegin, xEnd, yBegin and Yend (which are 2 bytes each)
	size_t payloadSize = ballPointsIndex * sizeof(linePointSt);
	if (payloadSize > CAM_PACKET_PAYLOAD_SIZE) {
		printf(
				"ERROR     : cam %u analyze send %zu ball points of %zu bytes, total %zu bytes, which does not fit in tx buffer of %u bytes\n",
				camIndex, ballPointsIndex, sizeof(linePointSt), payloadSize, CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	ssize_t ballLineWidth = 0;
	for (size_t ii = 0; ii < ballPointsIndex; ii++) {
		txPacket.pl.u16[4 * ii] = ballPoints[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = ballPoints[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = ballPoints[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = ballPoints[ii].yEnd;
		ballLineWidth += ballPoints[ii].xEnd - ballPoints[ii].xBegin + 1; // for ball yBegin is the same as yEnd
	}
	if (ballLineWidth > (500)) { // one line is 800 pixels, so 1/4 line * 20 lines = 200*20 = 4000
#if defined(__arm__) || defined(__aarch64__)
			*gpio_set_ptr = (1 << GPIO_PIN_BALL);
#endif
		// printf("INFO      : cam %u total ball point width %zd \n", camIndex, ballLineWidth);

#ifdef LATENCY_TIME_MEASUREMENT
		if (latencyMeasureWait > 20) {
			// ignore the measurements during startup
			struct timeval tv;
			gettimeofday(&tv, NULL);
			uint64_t ballGetTime = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;

			if (ballGetTime > ledSetTime) {
				uint64_t ledBalDelta = ballGetTime - ledSetTime;
				if (ledBalDelta < minLedBalDelta) {
					minLedBalDelta = ledBalDelta;
					printf("INFO      : cam %u minimal led to ball time %6.1f ms\n", camIndex, 0.001 * minLedBalDelta);
					fflush(stdout);
				}
				if (ledBalDelta > maxLedBalDelta) {
					maxLedBalDelta = ledBalDelta;
					printf("INFO      : cam %u maximal led to ball time %6.1f ms\n", camIndex, 0.001 * maxLedBalDelta);
					fflush(stdout);
				}
				avgLedBalDelta += ledBalDelta;
				avgLedBalDeltaAmount++;
				avgLedShmDelta += shmSetTime - ledSetTime;
				if ((avgLedBalDeltaAmount % 128) == 8) {
					printf("INFO      : cam %u average led to shm time %7.1f ms average led to ball time %6.1f ms\n",
							camIndex, 0.001 * avgLedShmDelta / avgLedBalDeltaAmount,
							0.001 * avgLedBalDelta / avgLedBalDeltaAmount);
					fflush(stdout);
				}
				// check if led was set before the shm
				if (shmSetTime < ledSetTime) {
					printf("ERROR     : cam %u shmSetTime 0x%016llx before ledSetTime 0x%016llx delta %6.1f ms\n",
							camIndex, shmSetTime, ledSetTime, 0.001 * (ledSetTime - shmSetTime));
					fflush(stdout);
				}

			}
		} else {
			latencyMeasureWait++;
		} // if (latencyMeasureWait
#endif

	}

	txPacket.id = camIndex * 64 + 3;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char *) "ball points");
	pthread_mutex_unlock(&txMutex);

#if defined(__arm__) || defined(__aarch64__)
	*gpio_clear_ptr = (1 << GPIO_PIN_BALL);
#endif
}

#ifdef NONO
// the balls and far balls are combined in the same list
linePointSt ballFarPoints[POINTS_MAX_AMOUNT];// 8187, in normal cases about 50 line points will be stored
size_t ballFarPointsIndex = 0;// amount of found ball points

// send the far away ball points
inline void ballFarPointsSend() {
	// one ballFar point is : xBegin, xEnd, yBegin and Yend (which are 2 bytes each)
	size_t payloadSize = ballFarPointsIndex * sizeof(linePointSt);
	if (payloadSize > CAM_PACKET_PAYLOAD_SIZE) {
		printf(
				"ERROR     : cam %u analyze send %zu ballFar points of %zu bytes, total %zu bytes, which does not fit in tx buffer of %u bytes\n",
				camIndex, ballFarPointsIndex, sizeof(linePointSt), payloadSize, CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for (size_t ii = 0; ii < ballFarPointsIndex; ii++) {
		txPacket.pl.u16[4 * ii] = ballFarPoints[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = ballFarPoints[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = ballFarPoints[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = ballFarPoints[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 7;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char *) "ballFar points");
	pthread_mutex_unlock(&txMutex);
}
#endif

linePointSt obstaclePoints[POINTS_MAX_AMOUNT]; // 8187, in normal cases about 500 line points will be stored
size_t obstaclePointsIndex = 0; // amount of found obstacle points

// send the obstacle points
inline void obstaclePointsSend() {
	// one obstacle point is : xBegin, xEnd, yBegin and Yend (which are 2 bytes each)
	size_t payloadSize = obstaclePointsIndex * sizeof(linePointSt);
	if (payloadSize > CAM_PACKET_PAYLOAD_SIZE) {
		printf(
				"ERROR     : cam %u analyze send %zu obstacle points of %zu bytes, total %zu bytes, which does not fit in tx buffer of %u bytes\n",
				camIndex, obstaclePointsIndex, sizeof(linePointSt), payloadSize, CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for (size_t ii = 0; ii < obstaclePointsIndex; ii++) {
		txPacket.pl.u16[4 * ii] = obstaclePoints[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = obstaclePoints[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = obstaclePoints[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = obstaclePoints[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 4;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char *) "obstacle points");
	pthread_mutex_unlock(&txMutex);
}

typedef enum lineDetSetateEnum {
	LINE_DET_INIT = 0,
	LINE_DET_FLOOR_BEFORE,
	LINE_DET_TRANSFER_BEFORE,
	LINE_DET_ON_LINE,
	LINE_DET_TRANSFER_AFTER,
	LINE_DET_FLOOR_AFTER
} lineDetStateT;

// try to find a white line on the green floor and reject other white objects like the goal and goal net
// a line is white
// a line is located on the floor
// the floor is green (floor pixel)
// so a line is valid when first green, then white, and again green
// unfortunately there might be non white or non green pixels between the green and white transfer
// because of noise it is possible that not all pixels on the floor are marked is a floor (green)
// because of noise it is possible that not all pixels on the line are market as a line (white)
// so let's make the following assumption
// at least 5 out of 10 (green) floor pixels
// then maximal 10 transfer pixels of a non (green) floor or non (white) line pixel
// at least 2 of 6 (white) line pixels
// then maximal 10 transfer pixels of a non (green) floor or non (white) line pixel
// at least 5 of 10 (green) floor pixels
// when above condition is met,
// - send the start point of the white line and the end point of the white line
// - search for the next line
// Note: the exported "point" exists of 2 points (xBegin, XEnd) on the same line (y)
static inline void lineDetectionLongAxis(const bool init, const uint8_t colorDetection, const uint16_t xx,
		const uint16_t yy) {
	static size_t floorPixels = 0;
	static size_t linePixels = 0;
	static size_t transferPixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;
	static bool xEndCheckEnable = true;

	static lineDetStateT state = LINE_DET_INIT;

	if (init) {
		// new line, start with new search
		state = LINE_DET_INIT;
		floorPixels = 0;
		linePixels = 0;
		transferPixels = 0;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if (colorDetection == DET_FLOOR) {
		// increase the floor pixels and decrease the other pixel counters
		if (floorPixels < lineFloorWindowSize) {
			floorPixels++;
		}
		if (linePixels > 0) {
			linePixels--;
		}
		if (transferPixels > 0) {
			transferPixels--;
		}
	} else if (colorDetection == DET_LINE) {
		// increase the line pixels and decrease the other pixel counters
		if (floorPixels > 0) {
			floorPixels--;
		}
		if (linePixels < lineWindowSize) {
			linePixels++;
		}
		if (transferPixels > 0) {
			transferPixels--;
		}
	} else {
		// if it is not a floor pixel or line pixel it is classified as transfer pixel
		// increase the transfer pixel and decrease the other pixel counters
		if (floorPixels > 0) {
			floorPixels--;
		}
		if (linePixels > 0) {
			linePixels--;
		}
		if (transferPixels < lineTransferPixelsMax) {
			transferPixels++;
		}
	}

	// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch (state) {
	case LINE_DET_INIT:
		// we are nowhere, restart the search
		if (colorDetection == DET_FLOOR) {
			state = LINE_DET_FLOOR_BEFORE;
		}
		break;
	case LINE_DET_FLOOR_BEFORE:
		if (colorDetection == DET_FLOOR) {
			// floor pixel, keep this state
		} else if (colorDetection == DET_LINE) {
			// the line is only valid if there are enough floor pixels
			if (floorPixels >= lineFloorPixelsMin) {
				// enough floor pixels to change state to the line detection state
				state = LINE_DET_ON_LINE;
				xBegin = xx;
			} else {
				if (floorPixels == 0) {
					// not enough floor pixels detected, so this cannot be classified as a line, restart search
					state = LINE_DET_INIT;
				}
			}
		} else {
			// not a floor or line pixel, assume this is a transfer pixel
			if (floorPixels >= lineFloorPixelsMin) {
				// enough floor pixels to change state to the transfer state
				state = LINE_DET_TRANSFER_BEFORE;
			} else {
				if (floorPixels == 0) {
					// not enough floor pixels detected, so this cannot be classified as a transfer, restart search
					state = LINE_DET_INIT;
				}
			}
		}
		break;
	case LINE_DET_TRANSFER_BEFORE:
		if (colorDetection == DET_FLOOR) {
			// it was likely a noise pixel, so we are still on the floor
			state = LINE_DET_FLOOR_BEFORE; // go back one state
		} else if (colorDetection == DET_LINE) {
			// line pixels found, go to the line detection state
			state = LINE_DET_ON_LINE;
			xBegin = xx;
		} else {
			// no line or floor pixel detected
			if (transferPixels >= lineTransferPixelsMax) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			}
		}
		break;
	case LINE_DET_ON_LINE:
		if (colorDetection == DET_FLOOR) {
			// a floor pixel detected, this can be a floor pixel before the line or a floor pixel after the line
			if (linePixels >= linePixelsMin) {
				state = LINE_DET_FLOOR_AFTER; // proceed two states
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				state = LINE_DET_FLOOR_BEFORE; // go back two states
			}
		} else if (colorDetection == DET_LINE) {
			// line pixels, keep this state
		} else {
			// no line or floor pixel detected, this can be noise, the transfer area before the line or transfer area after the line
			if (linePixels == 0) {
				// not a lot of line pixels, go back to the transfer area before the line
				state = LINE_DET_TRANSFER_BEFORE; // go back one state
			} else if (linePixels >= linePixelsMin) {
				// enough line pixels, go to the transfer area after the line
				state = LINE_DET_TRANSFER_AFTER; // proceed one state
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				// likely a noise pixel, keep this state
			}
		}
		break;
	case LINE_DET_TRANSFER_AFTER:
		if (colorDetection == DET_FLOOR) {
			// reached the floor after the line
			state = LINE_DET_FLOOR_AFTER; // proceed one state
		} else if (colorDetection == DET_LINE) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back one state
		} else {
			// no line or floor pixel detected, check if the transfer area is not to wide
			if (transferPixels >= lineTransferPixelsMax) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			} else {
				// keep this state, and wait until we reach the floor or determine we are still on the line
			}
		}
		break;

		break;
	case LINE_DET_FLOOR_AFTER:
		if (colorDetection == DET_FLOOR) {
			// floor pixel, check if we are done
			if (floorPixels >= lineFloorPixelsMin) {
				// store the line pixel
				if ((xBegin > xEnd) && xEndCheckEnable) {
					printf(
							"ERROR     : cam %u x analyze begin line pixel %u larger then x end line pixel %u for line %u\n",
							camIndex, xBegin, xEnd, yy);
					xEndCheckEnable = false; // TODO: investigate why this happens!, likely get worse when thresholds are set to zero
				} else {
					foundPointsLongAxis[linePointsIndexLongAxis].xBegin = xBegin;
					foundPointsLongAxis[linePointsIndexLongAxis].xEnd = xEnd;
					foundPointsLongAxis[linePointsIndexLongAxis].yBegin = yy;
					foundPointsLongAxis[linePointsIndexLongAxis].yEnd = yy;
					if (linePointsIndexLongAxis < POINTS_MAX_AMOUNT) {
						linePointsIndexLongAxis++; // prevent writing out of the buffer
					}
				}

				state = LINE_DET_INIT; // restart search for next line
			} // else keep this state
		} else if (colorDetection == DET_LINE) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back two states
		} else {
			// likely it was a noise pixel and we are in the transfer area
			state = LINE_DET_TRANSFER_AFTER; // go back two states
		}
		break;
	}
}

// functionally same as above, but for detection on the short axis
static inline void lineDetectionShortAxis(const bool init, const uint8_t colorDetection, const uint16_t xx,
		const uint16_t yy) {
	static size_t floorPixels = 0;
	static size_t linePixels = 0;
	static size_t transferPixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;
	static bool xEndCheckEnable = true;

	static lineDetStateT state = LINE_DET_INIT;

	if (init) {
		// new line, start with new search
		state = LINE_DET_INIT;
		floorPixels = 0;
		linePixels = 0;
		transferPixels = 0;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if (colorDetection == DET_FLOOR) {
		// increase the floor pixels and decrease the other pixel counters
		if (floorPixels < lineFloorWindowSize) {
			floorPixels++;
		}
		if (linePixels > 0) {
			linePixels--;
		}
		if (transferPixels > 0) {
			transferPixels--;
		}
	} else if (colorDetection == DET_LINE) {
		// increase the line pixels and decrease the other pixel counters
		if (floorPixels > 0) {
			floorPixels--;
		}
		if (linePixels < lineWindowSize) {
			linePixels++;
		}
		if (transferPixels > 0) {
			transferPixels--;
		}
	} else {
		// if it is not a floor pixel or line pixel it is classified as transfer pixel
		// increase the transfer pixel and decrease the other pixel counters
		if (floorPixels > 0) {
			floorPixels--;
		}
		if (linePixels > 0) {
			linePixels--;
		}
		if (transferPixels < lineTransferPixelsMax) {
			transferPixels++;
		}
	}

	// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch (state) {
	case LINE_DET_INIT:
		// we are nowhere, restart the search
		if (colorDetection == DET_FLOOR) {
			state = LINE_DET_FLOOR_BEFORE;
		}
		break;
	case LINE_DET_FLOOR_BEFORE:
		if (colorDetection == DET_FLOOR) {
			// floor pixel, keep this state
		} else if (colorDetection == DET_LINE) {
			// the line is only valid if there are enough floor pixels
			if (floorPixels >= lineFloorPixelsMin) {
				// enough floor pixels to change state to the line detection state
				state = LINE_DET_ON_LINE;
				xBegin = xx;
			} else {
				if (floorPixels == 0) {
					// not enough floor pixels detected, so this cannot be classified as a line, restart search
					state = LINE_DET_INIT;
				}
			}
		} else {
			// not a floor or line pixel, assume this is a transfer pixel
			if (floorPixels >= lineFloorPixelsMin) {
				// enough floor pixels to change state to the transfer state
				state = LINE_DET_TRANSFER_BEFORE;
			} else {
				if (floorPixels == 0) {
					// not enough floor pixels detected, so this cannot be classified as a transfer, restart search
					state = LINE_DET_INIT;
				}
			}
		}
		break;
	case LINE_DET_TRANSFER_BEFORE:
		if (colorDetection == DET_FLOOR) {
			// it was likely a noise pixel, so we are still on the floor
			state = LINE_DET_FLOOR_BEFORE; // go back one state
		} else if (colorDetection == DET_LINE) {
			// line pixels found, go to the line detection state
			state = LINE_DET_ON_LINE;
			xBegin = xx;
		} else {
			// no line or floor pixel detected
			if (transferPixels >= lineTransferPixelsMax) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			}
		}
		break;
	case LINE_DET_ON_LINE:
		if (colorDetection == DET_FLOOR) {
			// a floor pixel detected, this can be a floor pixel before the line or a floor pixel after the line
			if (linePixels >= linePixelsMin) {
				state = LINE_DET_FLOOR_AFTER; // proceed two states
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				state = LINE_DET_FLOOR_BEFORE; // go back two states
			}
		} else if (colorDetection == DET_LINE) {
			// line pixels, keep this state
		} else {
			// no line or floor pixel detected, this can be noise, the transfer area before the line or transfer area after the line
			if (linePixels == 0) {
				// not a lot of line pixels, go back to the transfer area before the line
				state = LINE_DET_TRANSFER_BEFORE; // go back one state
			} else if (linePixels >= linePixelsMin) {
				// enough line pixels, go to the transfer area after the line
				state = LINE_DET_TRANSFER_AFTER; // proceed one state
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				// likely a noise pixel, keep this state
			}
		}
		break;
	case LINE_DET_TRANSFER_AFTER:
		if (colorDetection == DET_FLOOR) {
			// reached the floor after the line
			state = LINE_DET_FLOOR_AFTER; // proceed one state
		} else if (colorDetection == DET_LINE) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back one state
		} else {
			// no line or floor pixel detected, check if the transfer area is not to wide
			if (transferPixels >= lineTransferPixelsMax) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			} else {
				// keep this state, and wait until we reach the floor or determine we are still on the line
			}
		}
		break;

		break;
	case LINE_DET_FLOOR_AFTER:
		if (colorDetection == DET_FLOOR) {
			// floor pixel, check if we are done
			if (floorPixels >= lineFloorPixelsMin) {
				// store the line pixel
				if ((xBegin > xEnd) && xEndCheckEnable) {
					printf(
							"ERROR     : cam %u x analyze begin line pixel %u larger then x end line pixel %u for line %u\n",
							camIndex, xBegin, xEnd, yy);
					xEndCheckEnable = false; // TODO: investigate why this happens!, likely get worse when thresholds are set to zero
				} else {
					foundPointsShortAxis[linePointsIndexShortAxis].xBegin = xBegin;
					foundPointsShortAxis[linePointsIndexShortAxis].xEnd = xEnd;
					foundPointsShortAxis[linePointsIndexShortAxis].yBegin = yy;
					foundPointsShortAxis[linePointsIndexShortAxis].yEnd = yy;
					if (linePointsIndexShortAxis < POINTS_MAX_AMOUNT) {
						linePointsIndexShortAxis++; // prevent writing out of the buffer
					}
				}

				state = LINE_DET_INIT; // restart search for next line
			} // else keep this state
		} else if (colorDetection == DET_LINE) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back two states
		} else {
			// likely it was a noise pixel and we are in the transfer area
			state = LINE_DET_TRANSFER_AFTER; // go back two states
		}
		break;
	}
}

inline void storeFloorPoint(const uint16_t xBegin, const uint16_t xEnd, const uint16_t yy) {
	if (xBegin > xEnd) {
		printf("ERROR     : cam %u analyze x begin pixel %u larger then x end pixel %u for floor point %u\n", camIndex,
				xBegin, xEnd, yy);
	} else {
		uint16_t length = xEnd - xBegin + 1;
		if (length >= 10) {
			// store floor point
			floorPoints[floorPointsIndex].xBegin = xBegin;
			floorPoints[floorPointsIndex].xEnd = xEnd;
			floorPoints[floorPointsIndex].yBegin = yy;
			floorPoints[floorPointsIndex].yEnd = yy;
			if (floorPointsIndex < POINTS_MAX_AMOUNT) {
				floorPointsIndex++; // prevent writing out of the buffer
			}
			// printf("INFO      : cam %u analyze store floor point %4zu xBegin %3u xEnd %3u y %3u\n", camIndex, floorPointsIndex,
			// 		xBegin, xEnd, yy);

		}
	}
}

static inline void floorDetection(const uint8_t colorDetection, const uint16_t xx, const uint16_t yy) {

	const uint16_t lost_max = 3;

	static bool foundPixel = false;
	static uint16_t xBegin;
	static uint16_t xEnd;
	static uint16_t lost;

	if (xx == 0) {
		// new line, start with new search
		foundPixel = false;
	}

	if (foundPixel) {
		// find out when we did not see a green pixel for a while
		if (colorDetection == DET_FLOOR) {
			// so we still on the green floor
			lost = 0;
			// update the last known position
			xEnd = xx;
		} else {
			if (lost > lost_max) {
				// so we did not see a green pixel for a while, so we are not on the green floor anymore
				// figure out if the collected data is worth storing
				storeFloorPoint(xBegin, xEnd, yy);
				foundPixel = false;
			}
			lost++; // we missed the green pixel
		}
	} else {
		// search for green pixel
		if (colorDetection == DET_FLOOR) {
			foundPixel = true;
			// remember where the green pixels started
			// TODO: add to list
			xBegin = xx;
			xEnd = xx;
			// keep track how long ago we saw a green pixel (we do not want to bail out after the first noise pixel)
			lost = 0;
		}

	}

	if ((xx == (FLOOR_WIDTH - 1)) && (foundPixel)) {
		// finalize a found pixel in case we are at the end of a line
		storeFloorPoint(xBegin, xEnd, yy);
		foundPixel = false;
		// printf("INFO      : cam %u analyze x begin pixel %u end pixel %u yy %u size %u for floor point\n", camIndex, xBegin, xEnd,
		//		yy, size);
	}
}

// ## yellow ball detection ##
// the ball can be (partly) on the (green) floor, (white) line, before (black) obstacle or
// before an arbitrary color when flying through they air
// so for ball detection we cannot use any of the other colors to reject wrong ball pixels
// search for a (yellow) ball pixels and continue until the last (yellow) ball pixel has been found
// there might be pixels in the ball area that are not classified as (yellow) ball pixels
// so we need to combine ball pixels when there are some non ball pixels in between
typedef enum ballDetSetateEnum {
	BALL_DET_SEARCH = 0, BALL_DET_FOUND, BALL_DET_GOOD_ENOUGH
} ballDetStateT;

static inline void ballDetection(const uint8_t colorDetection, const uint16_t xx, const uint16_t yy) {
	static uint16_t ballPixels = 0;
	static uint16_t ballFalsePixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static ballDetStateT state = BALL_DET_SEARCH;

	if (xx == 0) {
		// new line, start with new search
		ballPixels = 0;
		ballFalsePixels = 0;
		state = BALL_DET_SEARCH;
	}

// update the pixel counters, which are used by the state machine to determine next state
	if (((colorDetection & DET_BALL)) == DET_BALL) {
		if (ballPixels < ballWindowSize) {
			ballPixels++;
		}
		if (ballFalsePixels > 0) {
			ballFalsePixels--;
		}
	} else {
		if (ballPixels > 0) {
			ballPixels--;
		}
		if (ballFalsePixels < ballFalsePixelsMax) {
			ballFalsePixels++;
		}
	}

// first search for ball pixel, then keep on going until no ball pixel and finally send the ball pixels
	switch (state) {
	case BALL_DET_SEARCH:
		// we are nowhere, (re)start the search
		if ((colorDetection & DET_BALL) == DET_BALL) {
			state = BALL_DET_FOUND;
			xBegin = xx; // store the location where the ball is seen first
		}
		break;
	case BALL_DET_FOUND:
		if ((colorDetection & DET_BALL) == DET_BALL) {
			// still on the ball
			if (ballPixels >= ballPixelsMin) {
				// found enough ball pixels, now Classified as ball
				state = BALL_DET_GOOD_ENOUGH;
				xEnd = xx; // store the location where the ball is seen last
			}
		} else {
			// noise pixel, or noise on the ball, or lost the ball (beyond the ball)
			if (ballPixels == 0) {
				// not enough ball pixels, so this was likely was just a noise pixel
				state = BALL_DET_SEARCH;
			}
		}
		break;
	case BALL_DET_GOOD_ENOUGH:
		if ((colorDetection & DET_BALL) == DET_BALL) {
			// still on the ball
			xEnd = xx; // store the location where the ball is seen last
		} else {
			// noise on the ball, or lost the ball (beyond the ball)
			if (ballFalsePixels == ballFalsePixelsMax) {
				// not on the ball anymore, we are done with this ball
				ballPoints[ballPointsIndex].xBegin = xBegin;
				ballPoints[ballPointsIndex].xEnd = xEnd;
				ballPoints[ballPointsIndex].yBegin = yy;
				ballPoints[ballPointsIndex].yEnd = yy;
				if (ballPointsIndex < POINTS_MAX_AMOUNT) {
					ballPointsIndex++; // prevent writing out of the buffer
				}
				state = BALL_DET_SEARCH; // restart search for next ball
			} else {
				// wait for a number of non ball pixels (to be sure we are beyond the ball)
			}
		}
		break;
	}
}

// same as ballDetection function, except using color DET_BALL_FAR instead of DET_BALL
static inline void ballFarDetection(const uint8_t colorDetection, const uint16_t xx, const uint16_t yy) {
	static uint16_t ballFarPixels = 0;
	static uint16_t ballFarFalsePixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static ballDetStateT state = BALL_DET_SEARCH;

	if (xx == 0) {
		// new line, start with new search
		ballFarPixels = 0;
		ballFarFalsePixels = 0;
		state = BALL_DET_SEARCH;
	}

// update the pixel counters, which are used by the state machine to determine next state
	if ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR) {
		if (ballFarPixels < ballFarWindowSize) {
			ballFarPixels++;
		}
		if (ballFarFalsePixels > 0) {
			ballFarFalsePixels--;
		}
	} else {
		if (ballFarPixels > 0) {
			ballFarPixels--;
		}
		if (ballFarFalsePixels < ballFarFalsePixelsMax) {
			ballFarFalsePixels++;
		}
	}

// first search for ballFar pixel, then keep on going until no ballFar pixel and finally send the ballFar pixels
	switch (state) {
	case BALL_DET_SEARCH:
		// we are nowhere, (re)start the search
		if ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR) {
			state = BALL_DET_FOUND;
			xBegin = xx; // store the location where the ballFar is seen first
		}
		break;
	case BALL_DET_FOUND:
		if ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR) {
			// still on the ballFar
			if (ballFarPixels >= ballFarPixelsMin) {
				// found enough ballFar pixels, now Classified as ballFar
				state = BALL_DET_GOOD_ENOUGH;
				xEnd = xx; // store the location where the ballFar is seen last
			}
		} else {
			// noise pixel, or noise on the ballFar, or lost the ballFar (beyond the ballFar)
			if (ballFarPixels == 0) {
				// not enough ballFar pixels, so this was likely was just a noise pixel
				state = BALL_DET_SEARCH;
			}
		}
		break;
	case BALL_DET_GOOD_ENOUGH:
		if ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR) {
			// still on the ballFar
			xEnd = xx; // store the location where the ballFar is seen last
		} else {
			// noise on the ballFar, or lost the ballFar (beyond the ballFar)
			if (ballFarFalsePixels == ballFarFalsePixelsMax) {
				// not on the ballFar anymore, we are done with this ballFar
				ballPoints[ballPointsIndex].xBegin = xBegin;
				ballPoints[ballPointsIndex].xEnd = xEnd;
				ballPoints[ballPointsIndex].yBegin = yy;
				ballPoints[ballPointsIndex].yEnd = yy;
				if (ballPointsIndex < POINTS_MAX_AMOUNT) {
					ballPointsIndex++; // prevent writing out of the buffer
				}
				state = BALL_DET_SEARCH; // restart search for next ballFar
			} else {
				// wait for a number of non ballFar pixels (to be sure we are beyond the ballFar)
			}
		}
		break;
	}
}

#ifdef WHITE_BLACK_BALL_SEARCH
// ## white black ball detection ##
// Classify white black ball as valid as at least 5 white pixels and 2 black pixels
// the ball can be (partly) on the (green) floor, (white) line, before (black) obstacle or
// before an arbitrary color when flying through they air
// so for ball detection we cannot use any of the other colors to reject wrong ball pixels

// start searching for white pixels, continue until black pixel, continue until white pixel
// if enough white and black pixels, classify as ball
// there might be multiple white to black transfers
// continue until white pixel not seen for a while, then finalize ball and use last seen white pixel
// use the DET_LINE to indicate the white pixel and DET_OBSTACLE to indicate the black pixel
// because of noise it is allowed that 2 transfer pixels might occur before abandon the current ball search

static inline void ballWhiteBlackDetection(const colorDetectionT colorDetection, const uint16_t xx, const uint16_t yy) {
	static uint16_t ballWhitePixels = 0;
	static uint16_t ballBlackPixels = 0;
	static uint16_t ballFalsePixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static ballDetStateT state = BALL_DET_SEARCH;

	if (xx == 0) {
		// new line, start with new search
		ballWhitePixels = 0;
		ballBlackPixels = 0;
		ballFalsePixels = 0;
		state = BALL_DET_SEARCH;
	}

// update the pixel counters, which are used by the state machine to determine next state
	if (colorDetection == DET_LINE) {
		if (ballWhitePixels < ballWhiteWindowSize) {
			ballWhitePixels++;
		}
		if (ballFalsePixels > 0) {
			ballFalsePixels--;
		}
	} else if ( colorDetection == DET_OBSTACLE ) {
		if (ballBlackPixels < ballBlackWindowSize) {
			ballBlackPixels++;
		}
		if (ballFalsePixels > 0) {
			ballFalsePixels--;
		}
	} else {
		if (ballWhitePixels > 0) {
			ballWhitePixels--;
		}
		if (ballBlackPixels > 0) {
			ballBlackPixels--;
		}
		if (ballFalsePixels < ballWhiteBlackFalsePixelsMax) {
			ballFalsePixels++;
		}
	}

// start search for white pixels, then black pixels, then white pixels, if enough white and black finalize ball
	switch (state) {
		case BALL_DET_SEARCH:
		// we are nowhere, (re)start the search
		if ( ( colorDetection == DET_LINE ) && ( ballWhitePixels > 2 ) ) {
			// start when 2 white pixels are found, to prevent shade or black pixel of robot itself used as start point of the ball
			state = BALL_DET_FOUND;
			xBegin = xx - 1;// store the location where the ball is seen first (this is at least the second white pixel
		}
		break;
		case BALL_DET_FOUND:
		if ( ( colorDetection == DET_LINE || colorDetection == DET_OBSTACLE ) ) {
			// we are on white or black pixel of the ball
			if ( (ballWhitePixels >= ballWhitePixelsMin) && ( ballBlackPixels >= ballBlackPixelsMin ) ) {
				// found enough white and black pixels to classify as ball
				state = BALL_DET_GOOD_ENOUGH;
				xEnd = xx;// store the location where the ball is seen last
			}
		} else {
			// noise pixel, or noise on the ball, or lost the ball (beyond the ball)
			if (ballWhitePixels == 0) { // there are more white then black pixels on the ball, so use amount of white pixels to decide if we lost the ball
				// not enough ball pixels, so this was likely was just a noise pixel, start all over again
				state = BALL_DET_SEARCH;
			}
		}
		break;
		case BALL_DET_GOOD_ENOUGH:
		if ( (colorDetection == DET_LINE ) || ( colorDetection == DET_OBSTACLE ) ) {
			// still on the ball, determine the end of the ball
			xEnd = xx;// store the location where the ball is seen last
		} else {
			// noise on the ball, or lost the ball (beyond the ball)
			if (ballFalsePixels == ballWhiteBlackFalsePixelsMax) {
				// not on the ball anymore, we are done with this ball
				ballPoints[ballPointsIndex].xBegin = xBegin;
				ballPoints[ballPointsIndex].xEnd = xEnd;
				ballPoints[ballPointsIndex].yBegin = yy;
				ballPoints[ballPointsIndex].yEnd = yy;
				if (ballPointsIndex < POINTS_MAX_AMOUNT) {
					ballPointsIndex++; // prevent writing out of the buffer
				}
				state = BALL_DET_SEARCH; // restart search for next ball
			} else {
				// wait for a number of non ball pixels (to be sure we are beyond the ball)
			}
		}
		break;
	}
}
#endif

typedef enum obstacleDetSetateEnum {
	OBST_DET_INIT = 0, OBST_DET_OBSTACLE_PENDING, OBST_DET_BALL_FOUND, OBST_DET_OBSTACLE_FINALIZE, OBST_DET_TRANSFER,
} obstacleDetStateT;

// try to find obstacles on the floor
// obstacles are always in the floor
// so before the obstacle one of the following colors should be visible
//  - floor (green pixel)
//  - ball (yellow pixel)
//  - line (white pixel)
//  - for some robot's the robot itself (black pixel) (see robot 4)
// because of e.g. shades their might be an undefined color between the above list and the robot color
// this undefined color is treaded as part of the obstacle
// it also possible a ball is in front of the obstacle (robot) and blocking the obstacle color
// the ball pixels before an obstacle will also be treated as obstacle pixels
// so the obstacle starts (xbegin = closest by)
//  - obstacle color
//  - undefined color (transfer color)
//  - ball color

// to prevent fake obstacles a number of thresholds used to filter false obstacles e.g.
// at least 10 out of 20 floor pixels
// or at least 5 out of 10 ball pixels
// or at least 5 out of 10 line pixels
// then maximal 5 transfer pixels
// and then at least 5 out of 30 obstacle pixels
// then it is classified as obstacle

// when above condition is met:
// - send the start point of the obstacle and the end point of the obstacle
// Note: the exported "point" exists of 2 points (xBegin, XEnd) on the same line (y)
// Note: for obstacles that are not black, but e.g. red, just decrease
// the thresholds of the obstacle filter (increase the maximal obstacle saturation and and value)
// Note: on this level it still will be difficult to distinguish between black borders and obstacles
static inline void obstacleDetection(const uint8_t colorDetection, const uint16_t xx, const uint16_t yy) {
	static size_t floorPixels = 0;
	static size_t linePixels = 0;
	static size_t ballPixels = 0;
	static size_t obstaclePixels = 0;
	static size_t transferPixels = 0;

	static uint16_t xBegin;
	static uint16_t xEnd;

	static obstacleDetStateT state = OBST_DET_INIT;

	if (xx == OBSTACLE_X_START) {
		// new line, start with new search
		// assume we start with the black assembly of the robot itself
		state = OBST_DET_OBSTACLE_PENDING;
		xBegin = xx; // keep track of the first "found" obstacle pixel
		floorPixels = 0;
		linePixels = 0;
		ballPixels = 0;
		obstaclePixels = 0;
		transferPixels = 0;
	}

// update the pixel counters, which are used by the state machine to determine next state
	if (colorDetection == DET_FLOOR) {
		// increase the floor pixels
		if (floorPixels < obstacleFloorWindowSize) {
			floorPixels++;
		}
	} else {
		if (floorPixels > 0) {
			floorPixels--;
		}
	}

	if (colorDetection == DET_LINE) {
		// increase the line pixels
		if (linePixels < obstacleLineWindowSize) {
			linePixels++;
		}
	} else {
		if (linePixels > 0) {
			linePixels--;
		}
	}

	if (((colorDetection & DET_BALL) == DET_BALL) || ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR)) {
		// increase the ball pixels
		if (ballPixels < obstacleBallWindowSize) {
			ballPixels++;
		}
	} else {
		if (ballPixels > 0) {
			ballPixels--;
		}
	}
	if (colorDetection == DET_OBSTACLE) {
		// increase the obstacle pixels
		if (obstaclePixels < obstacleWindowSize) {
			obstaclePixels++;
		}
	} else {
		if (obstaclePixels > 0) {
			obstaclePixels--;
		}
	}

	if (colorDetection == DET_DEFAULT) {
		// if it is not a floor pixel, line pixel, ball pixel or obstacle pixel it
		// is classified as transfer pixel
		// increase the transfer pixel
		if (transferPixels < obstacleTransferPixelsMax) {
			transferPixels++;
		}
	} else {
		if (transferPixels > 0) {
			transferPixels--;
		}
	}

// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch (state) {
	case OBST_DET_INIT:
		if (colorDetection == DET_OBSTACLE) {
			// found an obstacle pixel, first check enough floor, line or ball pixels have been seen
			if ((floorPixels >= obstacleFloorPixelsMin) || (linePixels >= obstacleLinePixelsMin)
					|| (ballPixels >= obstacleBallPixelsMin)) {
				// we are on the floor, line or ball, so now we could expect an obstacle
				state = OBST_DET_OBSTACLE_PENDING;
				xBegin = xx; // keep track of the first found obstacle pixel
			}
		} else if (((colorDetection & DET_BALL) == DET_BALL) || ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR)) {
			// there might be a ball in front of the robot
			state = OBST_DET_BALL_FOUND;
			xBegin = xx; // keep track of the first found ball pixel, which might be in front of the robot
		} else if (colorDetection == DET_DEFAULT) {
			// no valid color, but this might be intermediate color between a known color and obstacle
			if ((floorPixels >= obstacleFloorPixelsMin) || (linePixels >= obstacleLinePixelsMin)
					|| (ballPixels >= obstacleBallPixelsMin)) {
				state = OBST_DET_TRANSFER;
				xBegin = xx; // keep track of the first found transfer pixel (which might be the start of the obstacle)
			}
		}
		break;
	case OBST_DET_TRANSFER:
		// we are in the transfer zone, this zone should not be to long, otherwise we have to go back to the init state
		if (colorDetection == DET_OBSTACLE) {
			// we just got lucky and straight found a obstacle, now go to the obstacle search state
			state = OBST_DET_OBSTACLE_PENDING;
		} else if (((colorDetection & DET_BALL) == DET_BALL) || ((colorDetection & DET_BALL_FAR) == DET_BALL_FAR)) {
			// there might be a ball in front of the robot
			state = OBST_DET_BALL_FOUND;
		} else {
			// still waiting for a valid obstacle pixel, which we need to jump to the obstacle search state
			if ((transferPixels == 0) || (transferPixels >= obstacleTransferPixelsMax)) {
				// to bad, no transfer pixel for a while or to many transfer pixels, in both cases start all over again
				state = OBST_DET_INIT;
			}
		}
		break;
	case OBST_DET_BALL_FOUND:
		if (colorDetection == DET_OBSTACLE) {
			// from ball color to obstacle color change, probably ball in front of robot
			state = OBST_DET_OBSTACLE_PENDING;
		} else {
			// noise pixel, or noise on the ball, or lost the ball (beyond the ball)
			if (ballPixels == 0) {
				// not enough ball pixels, so this was likely was just a noise pixel
				state = OBST_DET_INIT;
			}
		}
		break;
	case OBST_DET_OBSTACLE_PENDING:
		// determine if we have enough obstacle pixels, to be able to classify as obstacle
		if (colorDetection == DET_OBSTACLE) {
			if (obstaclePixels >= obstaclePixelsMin) {
				// printf( "obst pixels %3d obstaclePixelsMin %3d, obstaclePixels window %3d\n", obstaclePixels, obstaclePixelsMin,obstacleWindowSize );
				// found enough pixels, classify as obstacle
				state = OBST_DET_OBSTACLE_FINALIZE;
				xEnd = xx; // keep track of the last found obstacle pixel
			}
		} else {
			// no obstacle pixel anymore
			if (obstaclePixels == 0) {
				// did not see an obstacle pixel for a long while, start all over again
				state = OBST_DET_INIT;
			}
		}
		break;
	case OBST_DET_OBSTACLE_FINALIZE:
		// enough pixels to qualify as obstacle, now search for the end
		if (colorDetection == DET_OBSTACLE) {
			// still recognized as obstacle, keep state
			xEnd = xx; // keep track of the last found obstacle pixel
		} else {
			if (obstaclePixels == 0) {
				// did not see an obstacle pixel for a while, so we left the obstacle, restart the search
				state = OBST_DET_INIT;
			}
		}

		if ((xx == (FLOOR_WIDTH - 1)) || (obstaclePixels == 0)) {
			// reached the end of the line or did not see an obstacle for a while
			// now we also know for sure what the xEnd pixel was, store the pixel for transmission
			if ( ballPixels == 0 ) {
				// reject if the obstacle was actually a shade of the ball
				obstaclePoints[obstaclePointsIndex].xBegin = xBegin;
				obstaclePoints[obstaclePointsIndex].xEnd = xEnd;
				obstaclePoints[obstaclePointsIndex].yBegin = yy;
				obstaclePoints[obstaclePointsIndex].yEnd = yy;
				if (obstaclePointsIndex < POINTS_MAX_AMOUNT) { // prevent writing out of the buffer
					obstaclePointsIndex++;
				}
			}
		}
		break;
	}
}

// run this as 4 different threads
void* raspiColorDetection(void *arg) {
	int ii = *((int *) arg);
	free(arg);
	size_t bufferIndex = 3 * ii * ROI_WIDTH * ROI_HEIGHT / 4; // use different offset depending on thread
	size_t colorDetectionIndex = ii * ROI_WIDTH * ROI_HEIGHT / 4; // use different offset depending on thread
	size_t lineBegin = ii * ROI_HEIGHT / 4; // each thread is processing 1/4 of the 608 lines
	size_t lineEnd = lineBegin + ROI_HEIGHT / 4 - 1;
	// printf("INFO      : cam %u analyze color detection thread %d lines %3zu to %3zu\n", camIndex, ii, lineBegin, lineEnd);

	valueSum[ii] = 0; // start of new frame clear value sum of new frame
	for (size_t yy = lineBegin; yy <= lineEnd - 1; yy++) { // lines
		for (size_t xx = 0; xx < ROI_WIDTH; xx++) { // pixels
			uint8_t red = shmData[bufferIndex++];
			uint8_t green = shmData[bufferIndex++];
			uint8_t blue = shmData[bufferIndex++];

			// determine maximal value for red, green and blue
			uint32_t value = red;
			uint8_t value_type = 0;
			if (green > value) {
				value = green;
				value_type = 1; // green
			}
			if (blue > value) {
				value = blue;
				value_type = 2; // blue
			}

			// determine minimal value for red, green and blue
			uint32_t vmin = red;
			if (blue < vmin) {
				vmin = blue;
			}
			if (green < vmin) {
				vmin = green;
			}

			// determine the maximal color difference
			uint32_t vdelta = value - vmin;

			// determine saturation
			uint32_t saturation;
			if (value == 0) {
				saturation = 0;
			} else {
				saturation = (255 * vdelta) / value;
			}

			// determine hue
			// 0 to 360 degrees is represented as 0 to 180 degrees*2 so it fits in uint8_t
			int hue;
			if (vdelta == 0) {
				hue = 0;
			} else if (value_type == 0) {
				// cast to int32_t because green - blue can be negative and negative / uint32_t will cast the negative value to a +429496xxx
				hue = (30 * ((int32_t) green - (int32_t) blue)) / (int32_t) vdelta; // if Vmax = R,  -60 to 60 degrees -> -30 to 30 degrees*2
			} else if (value_type == 1) {
				// green
				hue = 60 + (30 * ((int32_t) blue - (int32_t) red)) / (int32_t) vdelta; // if Vmax = G,  60 to 180 degrees -> 30 to 90 degrees*2
			} else {
				// blue
				hue = 120 + (30 * ((int32_t) red - (int32_t) green)) / (int32_t) vdelta; // if Vmax = B, ( 180 to 300 degrees -> 80 to 150 degrees*2
			}
			if (hue < 0) {
				hue += 180;
			}
			valueSum[ii] += value;

			colorDetectionBuffer[colorDetectionIndex] = DET_DEFAULT;
			if ((value >= floorValMin) && (saturation >= floorSatMin) && (hue >= (int) floorHueMin)
					&& (hue <= (int) floorHueMax)) {
				colorDetectionBuffer[colorDetectionIndex] = DET_FLOOR;
			} else if ((value >= lineValMin) && (saturation <= lineSatMax)) {
				colorDetectionBuffer[colorDetectionIndex] = DET_LINE;
			} else {
				// TODO: add separate configuration for ball possession
				// do this only for cam0
				int ballSatMinTmp = ballSatMin;
				int ballHueMinTmp = ballHueMin;
				if (xx < 100) { // some robot's might have a large possession x offset (because of the metal)
					ballSatMinTmp = ballSatMin - 20; // TODO: quick hack
					ballHueMinTmp = ballHueMin - 5; // TODO: quick hack, required to recognize ball in ball handler
				}
				// the ball and ball far are not mutual exclusive
				if ((value >= ballValMin) && ((int)saturation >= ballSatMinTmp) && (hue >= ballHueMinTmp)
						&& (hue <= (int) ballHueMax)) {
					colorDetectionBuffer[colorDetectionIndex] = DET_BALL;
				}
				if ((value >= ballFarValMin) && (saturation >= ballFarSatMin) && (hue >= (int) ballFarHueMin)
						&& (hue <= (int) ballFarHueMax)) {
					colorDetectionBuffer[colorDetectionIndex] |= DET_BALL_FAR;
				}
			}

			if (colorDetectionBuffer[colorDetectionIndex] == DET_DEFAULT) {
				// color not identified, try some more
				if ((value <= obstacleValMax) && (saturation <= obstacleSatMax)) {
					colorDetectionBuffer[colorDetectionIndex] = DET_OBSTACLE;
				} else if ((value >= cyanValMin) && (saturation >= cyanSatMin) && (hue >= (int) cyanHueMin)
						&& (hue <= (int) cyanHueMax)) {
					colorDetectionBuffer[colorDetectionIndex] = DET_CYAN;
				}
			}
			colorDetectionIndex++;
		}
	}
	return NULL;
}

void* raspiBallDetection(void *arg) {
	(void) arg;
	size_t colorDetectionIndex = 0;
	ballPointsIndex = 0; // start with an empty line point list
	for (size_t yy = 0; yy < ROI_HEIGHT - 1; yy++) { // lines
		for (size_t xx = 0; xx < ROI_WIDTH; xx++) { // pixels
			// search for the yellow ball
			ballDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
			// search for the yellow ballFar
			ballFarDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
#ifdef WHITE_BLACK_BALL_SEARCH
			// search for the white and black ball
			ballWhiteBlackDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
#endif
			colorDetectionIndex++;
		}
	}
#ifdef CALCULATION_TIME_MEASUREMENT
	struct timeval tv;
	gettimeofday(&tv, NULL);
	ballSetTime = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;
#endif
	ballPointsSend();
	return NULL;
}

#ifdef NONO
// is ballFarDetection is already called from raspiBallDetection (so we can make use of the same ball list)
void* raspiBallFarDetection(void *arg) {
	(void) arg;
	size_t colorDetectionIndex = 0;
	ballFarPointsIndex = 0; // start with an empty line point list
	for (size_t yy = 0; yy < ROI_HEIGHT - 1; yy++) { // lines
		for (size_t xx = 0; xx < ROI_WIDTH; xx++) { // pixels
			// search for the yellow ballFar
			ballFarDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
			colorDetectionIndex++;
		}
	}
	ballFarPointsSend();
	return NULL;
}
#endif

void* raspiFloorDetection(void *arg) {
	(void) arg;
	size_t colorDetectionIndex = 0;
	floorPointsIndex = 0; // start with an empty floor point list
	for (size_t yy = 0; yy < ROI_HEIGHT - 1; yy++) { // lines (short camera axis)
		// floor pixels do not need to be detected in the air, only scan the lower part of the camera
		for (size_t xx = 0; xx < FLOOR_WIDTH; xx++) { // pixels (long camera axis)
			floorDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
			colorDetectionIndex++;
		}
		// correct the index for the skipped part at the end of the line (400 to 800)
		colorDetectionIndex += ROI_WIDTH - FLOOR_WIDTH;
	}
	floorPointsSend();
	return NULL;
}

void* raspiObstacleDetection(void *arg) {
	(void) arg;
	size_t colorDetectionIndex = 0;
	obstaclePointsIndex = 0; // start with an empty line point list
	for (size_t yy = 0; yy < ROI_HEIGHT - 1; yy++) { // lines (short camera axis)
		// obstacle pixels do not need to be detected in the air, only scan the lower part of the camera
		colorDetectionIndex+= OBSTACLE_X_START;
		for (size_t xx = OBSTACLE_X_START; xx < FLOOR_WIDTH; xx++) { // pixels (long camera axis)
			obstacleDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
			colorDetectionIndex++;
		}
		// correct the index for the skipped part at the end of the line (400 to 800)
		colorDetectionIndex += ROI_WIDTH - FLOOR_WIDTH;
	}
	obstaclePointsSend();
	return NULL;
}

void* raspiLineDetectionLongAxis(void *arg) {
	(void) arg;
	linePointsIndexLongAxis = 0; // start with an empty line point list
	size_t colorDetectionIndex = 0;

	//printf("INFO      : cam %u analyze line detection long camera axis of one frame\n", camIndex);

	// "vertical" scanning (long axis of the camera)
	// scan in the normal (left to right) direction (pixels on one line)
	// Note: the camera is 90 degrees rotated on the robot
	colorDetectionIndex = 0;
	for (size_t yy = 0; yy < ROI_HEIGHT - 1; yy++) { // lines (short camera axis)
		// line points do not need to be detected in the air, only scan the lower part of the camera
		for (size_t xx = 0; xx < FLOOR_WIDTH; xx++) { // pixels (long camera axis)
			// do not scan on every camera line for a field line to prevent that the ones scanned in the y direction
			// are suppressed (there is a limit on the amount of line points used by the solver)
			if ((yy % 16) == 8) {
				lineDetectionLongAxis(xx == 0, colorDetectionBuffer[colorDetectionIndex], xx, yy);
			}
			colorDetectionIndex++;
		}
		// correct the index for the skipped part at the end of the line (400 to 800)
		colorDetectionIndex += ROI_WIDTH - FLOOR_WIDTH;
	}
	linePointsSendLongAxis();
	return NULL;
}

void* raspiLineDetectionShortAxis(void *arg) {
	(void) arg;
	linePointsIndexShortAxis = 0; // start with an empty line point list
	size_t colorDetectionIndex = 0;

	//printf("INFO      : cam %u analyze line detection short camera axix of one frame\n", camIndex);

	// "horizontal" scanning (short axis of the camera)
	// to be able to find floor lines parallel with the camera scan lines, scan the "lines" from
	// top to bottom (instead left to right)
	// Note: the camera is 90 degrees rotated on the robot

	// this is a quite expensive operation because the read pointer is jumping through the memory instead
	// of a linear read

	// the floor lines are typically at the center of the camera image, so we do not need to scan the full
	// vertical line
	// the effective field width (long axis of the camera) is 400, so let's go for about 350
	// jumps of 20 give enough points for the top to bottom scanning

	for (uint16_t xx = 20; xx < 350; xx += 20) { // pixels : full range is 0:799
		// process one line from top to bottom
		// scan next vertical line
		colorDetectionIndex = xx; // reset pointer to pixel xx on the first line
		for (uint16_t yy = 0; yy < ROI_HEIGHT - 1; yy++) { // lines : 0:607 range
			lineDetectionShortAxis(yy == 0, colorDetectionBuffer[colorDetectionIndex], yy, xx); // swapped xx and yy to be able to re-use the lineDetection function, corrected after this loop
			colorDetectionIndex += ROI_WIDTH; // + 800
		}
	}
	// done with getting the line points in the "horizontal" (top to bottom) direction

	// the lineDetection function is designed for "vertical" (left to right) direction, but used
	// for the "horizontal" (top to bottom) direction, correct the orientation of the found
	// points by swapping the x and y in the list
	for (size_t ii = 0; ii < linePointsIndexShortAxis; ii++) {
		// swap xx and yy because we scanned in the other direction
		// Note: because we are scanning on the x-axis the x begin and x end are the same
		uint16_t tmp = foundPointsShortAxis[ii].xBegin;
		foundPointsShortAxis[ii].xBegin = foundPointsShortAxis[ii].yBegin;
		foundPointsShortAxis[ii].yBegin = tmp;
		tmp = foundPointsShortAxis[ii].xEnd;
		foundPointsShortAxis[ii].xEnd = foundPointsShortAxis[ii].yEnd;
		foundPointsShortAxis[ii].yEnd = tmp;
		int width = foundPointsShortAxis[ii].yEnd - foundPointsShortAxis[ii].yBegin;
		if (width < 0) {
			printf("ERROR     : cam %u analyze line points short axis y width is is %d but should be 0 or larger\n",
					camIndex, width);
		}
	}
	linePointsSendShortAxis();
	return NULL;
}

void* raspiStatistics(void *arg) {
	(void) arg;
	statisticsSend();
	return NULL;
}

void* raspiConfig(void *arg) {
	(void) arg;
	while (keepRunning) {
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		packetT rxPacket;
		// block until data received
		ssize_t nBytes = recvfrom(multReceiveFd, &rxPacket, sizeof(rxPacket), 0, (struct sockaddr *) &fromAddr,
				&fromAddrlen);
		if (nBytes < 0) {
			printf("ERROR     : cam %u analyze reading from socket, %s", camIndex, strerror(errno));
			fflush(stdout);
			close(multReceiveFd);
			exit(EXIT_FAILURE);
		}

		bool valid = true;
		if (rxPacket.size != nBytes) {
			printf("ERROR     : cam %u analyze packet incomplete, received %zd bytes, but expected %u bytes\n",
					camIndex, nBytes, rxPacket.size);
			fflush(stdout);
			valid = false;
		} else {
			//printf("INFO      : received %u bytes from sender %s:%u\n", rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//		ntohs(fromAddr.sin_port));
		}

		if (valid) {
			if (rxPacket.id == 31) {
				size_t expected = 76;
#ifdef WHITE_BLACK_BALL_SEARCH
				expected += 10;
#endif
				if (rxPacket.size != expected) {
					printf(
							"ERROR     : cam %u analyze configuration packet should be %zu bytes, but received %u bytes\n",
							camIndex, expected, rxPacket.size);
					fflush(stdout);
				} else {

					size_t ii = 0;
					lineValMin = rxPacket.pl.u16[ii++];
					lineSatMax = rxPacket.pl.u16[ii++];
					lineTransferPixelsMax = rxPacket.pl.u16[ii++];
					lineFloorWindowSize = rxPacket.pl.u16[ii++];
					lineFloorPixelsMin = rxPacket.pl.u16[ii++];
					lineWindowSize = rxPacket.pl.u16[ii++];
					linePixelsMin = rxPacket.pl.u16[ii++];

					ballValMin = rxPacket.pl.u16[ii++];
					ballSatMin = rxPacket.pl.u16[ii++];
					ballHueMin = rxPacket.pl.u16[ii++];
					ballHueMax = rxPacket.pl.u16[ii++];
					ballWindowSize = rxPacket.pl.u16[ii++];
					ballPixelsMin = rxPacket.pl.u16[ii++];
					ballFalsePixelsMax = rxPacket.pl.u16[ii++];

					ballFarValMin = rxPacket.pl.u16[ii++];
					ballFarSatMin = rxPacket.pl.u16[ii++];
					ballFarHueMin = rxPacket.pl.u16[ii++];
					ballFarHueMax = rxPacket.pl.u16[ii++];
					ballFarWindowSize = rxPacket.pl.u16[ii++];
					ballFarPixelsMin = rxPacket.pl.u16[ii++];
					ballFarFalsePixelsMax = rxPacket.pl.u16[ii++];

#ifdef WHITE_BLACK_BALL_SEARCH
					ballWhiteWindowSize = rxPacket.pl.u16[ii++];
					ballWhitePixelsMin = rxPacket.pl.u16[ii++];
					ballBlackWindowSize = rxPacket.pl.u16[ii++];
					ballBlackPixelsMin = rxPacket.pl.u16[ii++];
					ballWhiteBlackFalsePixelsMax = rxPacket.pl.u16[ii++];
#endif

					floorValMin = rxPacket.pl.u16[ii++];
					floorSatMin = rxPacket.pl.u16[ii++];
					floorHueMin = rxPacket.pl.u16[ii++];
					floorHueMax = rxPacket.pl.u16[ii++];

					obstacleValMax = rxPacket.pl.u16[ii++];
					obstacleSatMax = rxPacket.pl.u16[ii++];
					obstacleFloorWindowSize = rxPacket.pl.u16[ii++];
					obstacleFloorPixelsMin = rxPacket.pl.u16[ii++];
					obstacleLineWindowSize = rxPacket.pl.u16[ii++];
					obstacleLinePixelsMin = rxPacket.pl.u16[ii++];
					obstacleBallWindowSize = rxPacket.pl.u16[ii++];
					obstacleBallPixelsMin = rxPacket.pl.u16[ii++];
					obstacleTransferPixelsMax = rxPacket.pl.u16[ii++];
					obstacleWindowSize = rxPacket.pl.u16[ii++];
					obstaclePixelsMin = rxPacket.pl.u16[ii++];
					continueProcessOneFrame = true; // for testing on x86_64
				}
			} else if (rxPacket.id == 34) {
				// analyze process control
				size_t expected = 6;
				if (rxPacket.size != expected) {
					printf("ERROR     : cam %u analyze control packet should be %zu bytes, but received %u bytes\n",
							camIndex, expected, rxPacket.size);
					fflush(stdout);
				} else {
					//printf("INFO      : cam %u analyze received analyze control packet of %u bytes\n", camIndex,
					//		rxPacket.size);
					//fflush(stdout);
					size_t ii = 0;
					imageSendFrames = rxPacket.pl.u16[ii++];
				}
			} else if (rxPacket.id == 32) {
				// packet for grabber process
			} else if (rxPacket.id == 64) {
				// packet for system process
			} else if (rxPacket.id == 67) {
				// packet for system process pixels offset
			} else if (rxPacket.id == 68) {
				// packet for to exit raspi system process
			} else if (rxPacket.id == 69) {
				// packet to synchronize frame counter of cam1, cam2 and cam3 with frame counter of cam0
			} else if (rxPacket.id == 98) {
				if (rxPacket.pl.u32[0] == 0xdead0011) {
					printf("WARNING   : cam %u analyze received poweroff\n", camIndex);
					fflush(stdout);
					if (system("sudo /sbin/poweroff") <= 0) {
						printf("ERROR     : cam %u analyze cannot perform \"sudo /sbin/poweroff\", message: %s\n",
								camIndex, strerror(errno));
						fflush(stdout);
						exit(EXIT_FAILURE);
					}
				}
			} else if (rxPacket.id == 99) {
				if (rxPacket.pl.u32[0] == 0xdead0022) {
					printf("WARNING   : cam %u analyze received reboot\n", camIndex);
					fflush(stdout);
					if (system("sudo /sbin/reboot") <= 0) {
						printf("ERROR     : cam %u analyze cannot perform \"sudo /sbin/reboot\", message: %s\n",
								camIndex, strerror(errno));
						fflush(stdout);
						exit(EXIT_FAILURE);
					}
				}
			} else if (rxPacket.id == 100) {
				if (rxPacket.pl.u32[0] == 0xdead0033) {
					printf("WARNING   : cam %u analyze received exit application\n", camIndex);
					fflush(stdout);
					keepRunning = false;
				}
			} else {
				printf("ERROR     : cam %u analyze received invalid packet id %u\n", camIndex, rxPacket.id);
			} // if (rx.packet.id
		} // if (valid)
	} // while (true)

	return NULL;
}

void* gpioLedToggle(void *arg) {
	(void) arg;
	while (keepRunning) {
		usleep(128000); // 128ms, much longer then one camera frame of 25ms

#ifdef LATENCY_TIME_MEASUREMENT
		struct timeval tv;
		gettimeofday(&tv, NULL);
		ledSetTime = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;
#endif

#if defined(__arm__) || defined(__aarch64__)
		*gpio_set_ptr = (1 << GPIO_PIN_LED);
#endif

		usleep(0); // scope measurement shows becomes pulse between 60 and 100 us

#if defined(__arm__) || defined(__aarch64__)
		*gpio_clear_ptr = (1 << GPIO_PIN_LED);
#endif

	} // while (true)

	return NULL;
}

inline void processOneFrame() {
	// printf("INFO      : cam %u analyze process one frame\n", camIndex);

	for (size_t ii = 0; ii < 4; ii++) {
		int *arg2 = (int *) malloc(sizeof(*arg2)); // the free is performed in the raspiColorDetection function
		if (arg2 == NULL) {
			printf("ERROR     : cam %u analyze cannot allocate memory for thread argument\n", camIndex);
			exit(EXIT_FAILURE);
		}

		*arg2 = ii;

		int err = pthread_create(&(raspiColorDetectionThreadId[ii]), NULL, &raspiColorDetection, (void *) arg2);
		if (err != 0) {
			printf("ERROR     : cam %u analyze cannot start raspiColorDetection thread, %s\n", camIndex, strerror(err));
		} // if (err
	} // for

	for (size_t ii = 0; ii < 4; ii++) {
		pthread_join(raspiColorDetectionThreadId[ii], NULL);
	}

	int err = pthread_create(&(raspiBallDetectionThreadId), NULL, &raspiBallDetection, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start ball detection thread, %s\n", camIndex, strerror(err));
	}
#ifdef NONO
	err = pthread_create(&(raspiBallFarDetectionThreadId), NULL, &raspiBallFarDetection, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start ballFar detection thread, %s\n", camIndex, strerror(err));
	}
#endif

	err = pthread_create(&(raspiStatisticsThreadId), NULL, &raspiStatistics, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start statistics thread, %s\n", camIndex, strerror(err));
	}

	err = pthread_create(&(raspiFloorDetectionThreadId), NULL, &raspiFloorDetection, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start floor detection thread, %s\n", camIndex, strerror(err));
	}

	err = pthread_create(&(raspiLineDetectionLongAxisThreadId), NULL, &raspiLineDetectionLongAxis, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start long axis line detection thread, %s\n", camIndex,
				strerror(err));
	}

	err = pthread_create(&(raspiLineDetectionShortAxisThreadId), NULL, &raspiLineDetectionShortAxis, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start short axis line detection thread, %s\n", camIndex,
				strerror(err));
	}
	err = pthread_create(&(raspiObstacleDetectionThreadId), NULL, &raspiObstacleDetection, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start obstacle detection thread, %s\n", camIndex, strerror(err));
	}

	pthread_join(raspiBallDetectionThreadId, NULL);
	// pthread_join(raspiBallFarDetectionThreadId, NULL);
	pthread_join(raspiLineDetectionLongAxisThreadId, NULL);
	pthread_join(raspiLineDetectionShortAxisThreadId, NULL);
	pthread_join(raspiObstacleDetectionThreadId, NULL);
	pthread_join(raspiFloorDetectionThreadId, NULL);
	pthread_join(raspiStatisticsThreadId, NULL);
	// timing critical stuff done, now send a part of the image (if requested)
	// imageSendConditional();
}

int main(int argc, char** argv) {

	int opt = 0;
	int runTime = -1;

	while ((opt = getopt(argc, argv, "hl:t:")) != -1) {
		switch (opt) {
		case 'h':
			printf("INFO      : -l camera index\n");
			printf("          : -t run time in seconds (optional)\n");
			return 0;
			break;
		case 'l':
			camIndex = atoi(optarg);
			break;
		case 't':
			runTime = atoi(optarg);
			break;
		}
	}
	printf("INFO      : cam %u analyze started\n", camIndex);

	// get process id which is used to determine raspi analyze application uptime
	processId = getpid();

	if (processId == 0) {
		printf("ERROR     : cam %u analyze application process ID cannot be %d\n", camIndex, processId);
		fflush(stdout);
		exit( EXIT_FAILURE);
	} else {
		printf("INFO      : cam %u analyze application process ID %d\n", camIndex, processId);
	}

	// get the time the application was started (since CPU is up and running) (to determine application uptime)
	char proc_pid_stat[64];
	sprintf(proc_pid_stat, "/proc/%d/stat", processId);

	FILE *fp = NULL;
	fp = fopen(proc_pid_stat, "r");  // r for read
	if (fp == NULL) {
		printf("ERROR     : cam %u analyze application %s when opening file %s for reading\n", camIndex,
				strerror(errno), proc_pid_stat);
		fflush(stdout);
		exit( EXIT_FAILURE);
	}

	// read the application start time from /proc/[pid/stat element 22
	char tmp_s[128];
	char tmp_c;
	int tmp_d;
	unsigned int tmp_u;
	unsigned long int tmp_lu;
	long int tmp_ld;
	unsigned long long int startTime;

	//                        1  2  3  4  5  6  7  8  9  10  11  12  13  14  15  16  17  18  19  20  21   22
	int retVal = fscanf(fp, "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %ld %llu ...",

	//           1      2       3       4       5       6       7       8       9       10       11       12
			&tmp_d, tmp_s, &tmp_c, &tmp_d, &tmp_d, &tmp_d, &tmp_d, &tmp_d, &tmp_u, &tmp_lu, &tmp_lu, &tmp_lu,

			//   13       14       15       16       17       18       19       20       21          22
			&tmp_lu, &tmp_lu, &tmp_lu, &tmp_ld, &tmp_ld, &tmp_ld, &tmp_ld, &tmp_ld, &tmp_ld, &startTime);

	if (retVal <= 0) {
		printf("ERROR     : cam %u analyze extract values from /proc/%d/stat, message: %s\n", camIndex, processId,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// convert from clock ticks to seconds
	analyzeApplicationStartTime = 1.0 * startTime / sysconf(_SC_CLK_TCK);
	fclose(fp);

	multiCastSendSetup();
	// TODO: send message to x86_64 to indicate analyze process just started
	multiCastReceiveSetup();

#if defined(__arm__) || defined(__aarch64__)
	// create access to raspi GPIO control (for external timing measurement)
	int dev_mem;
	if ((dev_mem = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		printf("ERROR     : cam %u analyze cannot open /dev/mem, message %s\n", camIndex, strerror(errno));
		printf("WARNING   : cam %u analyze you need to run this process as root\n", camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else {
		printf("INFO      : cam %u analyze opened /dev/mem for read write\n", camIndex);
	}

	// cast void pointer to uint32_t so it is easier to add the offset
	uint32_t *gpio_ptr = (uint32_t*) mmap(NULL, 256, PROT_READ | PROT_WRITE, MAP_SHARED, dev_mem, GPIO_REGISTER_BASE);
	close(dev_mem);

	if (gpio_ptr == MAP_FAILED) {
		printf("ERROR     : cam %u analyze cannot map /dev/mem to pointer %p, message %s\n", camIndex,
				(void *) gpio_ptr, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else {
		printf("INFO      : cam %u analyze mapped /dev/mem to pointer %p\n", camIndex, (void *) gpio_ptr);
	}

	*(gpio_ptr + (GPIO_PIN_BALL / 10)) &= ~(7 << ((GPIO_PIN_BALL % 10) * 3)); // set as input to prepare
	*(gpio_ptr + (GPIO_PIN_BALL / 10)) |= (1 << ((GPIO_PIN_BALL % 10) * 3));// now we can set as output
	printf("INFO      : cam %u analyze BALL GPIO %d set to output\n", camIndex, GPIO_PIN_BALL);

	*(gpio_ptr + (GPIO_PIN_LED / 10)) &= ~(7 << ((GPIO_PIN_LED % 10) * 3));// set as input to prepare
	*(gpio_ptr + (GPIO_PIN_LED / 10)) |= (1 << ((GPIO_PIN_LED % 10) * 3));// now we can set as output
	printf("INFO      : cam %u analyze LED GPIO %d set to output\n", camIndex, GPIO_PIN_LED);

	gpio_set_ptr = gpio_ptr + (GPIO_SET_OFFSET / sizeof(uint32_t));
	printf("INFO      : cam %u analyze GPIO set pointer %p\n", camIndex, (void *) gpio_set_ptr);
	gpio_clear_ptr = gpio_ptr + (GPIO_CLR_OFFSET / sizeof(uint32_t));
	printf("INFO      : cam %u analyze GPIO clear pointer %p\n", camIndex, (void *) gpio_clear_ptr);

	int suppressCounter = 0;
	int shmFd = shm_open("/raspiGrab", O_RDONLY, 0666);
	if (shmFd < 0) {
		printf("ERROR     : cam %u analyze cannot open /dev/shm/raspiGrab with shm_open for reading, message %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

	shmData = (uint8_t *) mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shmFd, 0);
	printf("INFO      : cam %u analyze shared memory mapped to address: %p\n", camIndex, shmData);

	shmDataSend = (uint8_t *) malloc(SHM_SIZE); // for double buffering

#else
// x86_64 simulation mode

// determine time when application is started to be able to generate steady FPS
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double timeBase = tv.tv_sec + tv.tv_usec / 1000000.0;

// for x86_64 emulation mode the shared memory buffer is filled from a file instead of /dev/shm/raspiGrab
	shmData = (uint8_t *) malloc(SHM_SIZE);
	shmDataSend = (uint8_t *) malloc(SHM_SIZE); // for double buffering
// read the RGB data from file (each camera uses a different file)
	char rgbFile[256];
	sprintf(rgbFile, "cam%dImage.rgb", camIndex);

	struct stat rgbFileAttrPrev;
#endif // #if defined(__arm__) || defined(__aarch64__)

	int err = pthread_create(&(raspiConfigThreadId), NULL, &raspiConfig, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start raspiConfig thread, %s\n", camIndex, strerror(err));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	err = pthread_create(&(gpioLedToggleThreadId), NULL, &gpioLedToggle, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u analyze cannot start gpioLedToggle thread, %s\n", camIndex, strerror(err));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// flush all normal startup messages to the log file
	fflush(stdout);

	while (keepRunning) {

#if defined(__arm__) || defined(__aarch64__)
		// determine if there is new data in the shared memory buffer
		uint32_t *frameCounterPtr = (uint32_t *) shmData;
		if (*frameCounterPtr != frameCounterPrev) {
#ifdef LATENCY_TIME_MEASUREMENT
			struct timeval tv;
			gettimeofday(&tv, NULL);
			shmSetTime = (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;
#endif
			if ((*frameCounterPtr != (frameCounterPrev + 1)) && (frameCounterPrev != 0)) {
				if( suppressCounter == 0 ) {
					printf("WARNING   : cam %u analyze received frame counter %u but expected %u, suppress next 1000\n", camIndex, *frameCounterPtr, frameCounterPrev + 1);
					fflush(stdout);
					suppressCounter = 1000;
				}
				suppressCounter--;
			}
			frameCounterPrev = *frameCounterPtr;
#else

		// only read the rgb file (and perform the processing) when the rgb file has changed (to reduce the load for x86_64 emulation)
		struct stat rgbFileAttr;
		stat(rgbFile, &rgbFileAttr);
		if (rgbFileAttr.st_mtime != rgbFileAttrPrev.st_mtime) {
			stat(rgbFile, &rgbFileAttrPrev);
			FILE *readPtr;
			readPtr = fopen(rgbFile, "rb");  // r for read, b for binary
			if (readPtr == NULL) {
				printf("ERROR     : cam %u analyze %s when opening file %s for reading\n", camIndex, strerror(errno),
						rgbFile);
				fflush(stdout);
				exit( EXIT_FAILURE);
			} else {
				printf("INFO      : cam %u analyze using file %s as camera input\n", camIndex, rgbFile);
			}

			size_t readSize = fread(shmData, SHM_SIZE, 1, readPtr);
			if (readSize == 0) {
				printf("ERROR     : cam %u analyze read error from %s, %s\n", camIndex, rgbFile, strerror(errno));
				fflush(stdout);
				exit(EXIT_FAILURE);
			}

			fclose(readPtr);
			continueProcessOneFrame = true;
		}
#endif
#ifdef NONO
		// the copy of the raw debug image has been replaced by the sending raspiSystem jpg image
		// only copy the latest image to the send buffer, when the remote requested the image and there is no image transfer going on
		if ((imageSendFrames > 0) && (!imageSendBusy)) {
			// make use of double buffering for the transmitted image
			// prefer "old" data above fast data (especially for dewarp calibration)
			memcpy(&shmDataSend[0], &shmData[0], SHM_SIZE);
			imageSendBusy = true;// handshake to imageSend function to start sending the image
		}
#endif

		if (continueProcessOneFrame) {
			processOneFrame();
#ifdef CALCULATION_TIME_MEASUREMENT
			struct timeval tv;
			gettimeofday(&tv, NULL);
			uint64_t doneSetTime = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;
			if (ballSetTime < shmSetTime) {
				printf("ERROR     : cam %u ballSetTime 0x%016llx before shmSetTime 0x%016llx\n", camIndex, ballSetTime,
						shmSetTime);
				fflush(stdout);
			}
			if (doneSetTime < ballSetTime) {
				printf("ERROR     : cam %u doneSetTime 0x%016llx before ballSetTime 0x%016llx\n", camIndex, doneSetTime,
						ballSetTime);
				fflush(stdout);
			}

			static int printCounter = 0;
			if ((printCounter % 256) == 0) {
				uint64_t deltaTime = doneSetTime - shmSetTime;
				avgShmDoneDelta += deltaTime;
				deltaTime = ballSetTime - shmSetTime;
				avgShmBallDelta += deltaTime;
				avgShmDoneDeltaAmount++;
				printf("INFO      : cam %u avg analyze calc time %6.1f avg ball calc time %6.1f ms\n", camIndex,
						0.001 * avgShmDoneDelta / avgShmDoneDeltaAmount,
						0.001 * avgShmBallDelta / avgShmDoneDeltaAmount);
				fflush(stdout);
			}
			printCounter++;
#endif
		} else {
			// only for x86
			// just re-send the first calculated data
			ballPointsSend();
			linePointsSendLongAxis(); // triggers the update of the viewer on x86
			linePointsSendShortAxis();
			floorPointsSend(); // green floor
			obstaclePointsSend();
			statisticsSend();
			// imageSendConditional();
		}
#if defined(__arm__) || defined(__aarch64__)
	} else {
		// wait and poll again
		usleep(1);// values allowed up to 25 ms (40 fps = 25 ms)
	} // if (*frameCounterPtr != frameCounterPrev) {
#else
		continueProcessOneFrame = false; // image stays the same, so no recalculation required (reduce CPU load on x86)
		// x86_64 emulation
		// 5 FPS to keep the load on the x86_64 under control
#define FPS 10
		// create accurate FPS to verify the FPS accuracy calculation in receiver
		double timeNext = timeBase + 1.0 * frameCounterPrev / FPS;
		double timeCurrent = 0.0;
		while (timeCurrent <= timeNext) {
			usleep(1000); // check very millisecond
			gettimeofday(&tv, NULL);
			timeCurrent = tv.tv_sec + tv.tv_usec / 1000000.0;
		}
		if ((camIndex == 0) && ((frameCounterPrev % (5 * FPS)) == 0)) { // print only once every 5 seconds
			printf("INFO      : cam %u analyze frame %4u analyze uptime %6u\n", camIndex, frameCounterPrev,
					analyzeApplicationUptime);
		}
		if (runTime == 0) {
			keepRunning = false;
		}
		if (runTime > 0) {
			if ((frameCounterPrev % FPS) == 0) {
				runTime--; // decrease every second
			}
		}
		frameCounterPrev++;
#endif
	} // while (keepRunning)

#if defined(__arm__) || defined(__aarch64__)
	munmap(shmData, SHM_SIZE);
	free(shmDataSend);
	close(shmFd);
#else
	free(shmData);
	free(shmDataSend);
#endif

	// gracefully shutdown for valgrind report

	// nothing is send anymore to the x86_64
	multiCastSendClose();

#if ! defined(__arm__) && ! defined(__aarch64__)
	// send a stop command to the still running configuration receive thread
	// only for valgrind graceful shutdown on x86_64
	printf("INFO      : cam %u analyze send stop to configuration receive thread\n", camIndex);
	multiCastReceiveLoopBackSetup();
	multiCastReceiveLoopBackStop();
	multiCastReceiveLoopBackClose();
#endif

	// stop the configuration thread
	// wait until the raspi configuration thread has been stopped
	pthread_join(raspiConfigThreadId, NULL);

	// wait until the gpio led toggle thread has been stopped
	pthread_join(gpioLedToggleThreadId, NULL);

	// now we also can close the receive configuration port
	multiCastReceiveClose();

	printf("INFO      : cam %u analyze all done\n", camIndex);
	return EXIT_SUCCESS;
}
