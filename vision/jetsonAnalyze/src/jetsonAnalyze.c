// Copyright 2018-2021 Andre Pool
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

bool keepRunning = true; // for graceful main and thread shutdown
uint8_t camIndex = 0; // provided through command line
int multReceiveFd = 0;
int multReceiveLoopBackFd = 0;
int multSendFd = 0;
struct sockaddr_in toAddr;
uint32_t frameCounterPrev = 0;
float analyzeApplicationStartTime = 0;
uint16_t analyzeApplicationUptime = 0;
pid_t processId = 0;
uint32_t valueSum[4] = { 0, 0, 0, 0 }; // 4 threads, total sum of all values fit in 32 bits: 255*800*608 = 0x7582080

#define SHM_SIZE ( 3 * ROI_WIDTH * ROI_HEIGHT ) // 3 bytes per pixel
uint8_t *shmData = 0;
uint8_t *shmDataSend = 0;

typedef enum colorDetectionEnum {
	DET_DEFAULT = 0, DET_LINE = 1, DET_FLOOR = 32, DET_ZZ_LAST = 128
} colorDetectionT;

uint8_t colorDetectionBuffer[ROI_HEIGHT * ROI_WIDTH];
bool jetsonColorDetectionBusy = false;

pthread_t jetsonColorDetectionThreadId[4];
pthread_t jetsonFloorDetectionThreadId;
pthread_t jetsonLineDetectionLongAxisThreadId;
pthread_t jetsonLineDetectionShortAxisThreadId;
pthread_t jetsonStatisticsThreadId;
pthread_t jetsonConfigThreadId;

// line point detection
uint16_t lineValMin = LINE_VAL_MIN;
uint16_t lineSatMax = LINE_SAT_MAX;
uint16_t lineTransferPixelsMax = LINE_TRANSFER_PIXELS_MAX;
uint16_t lineFloorWindowSize = LINE_FLOOR_WINDOW_SIZE;
uint16_t lineFloorPixelsMin = LINE_FLOOR_PIXELS_MIN;
uint16_t lineWindowSize = LINE_WINDOW_SIZE;
uint16_t linePixelsMin = LINE_PIXELS_MIN;

// floor point detection
uint16_t floorValMin = FLOOR_VAL_MIN;
uint16_t floorSatMin = FLOOR_SAT_MIN;
uint16_t floorHueMin = FLOOR_HUE_MIN;
uint16_t floorHueMax = FLOOR_HUE_MAX;

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
} __attribute__((packed)) packetT;

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

void multiCastAddRoute() {
	FILE *fp = NULL;
	// delete existing route to lo (if exist), just ignore warning when not exists
	char routeDelCmd[] = "sudo route del -net 224.16.16.0 netmask 255.255.255.0 dev lo 2>/dev/null";
	fp = popen(routeDelCmd, "r");
	pclose(fp);

	// add route for multiCast packets, just ignore warning when already exists
#if defined(__aarch64__)
	char routeAddCmd[] = "sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev eth0 2>/dev/null";
#else
	char routeAddCmd[] = "sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6 2>/dev/null";
#endif
	fp = popen(routeAddCmd, "r");
	if( fp == NULL ) {
		printf("ERROR    cam %u analyze application cannot execute %s, message: %s\n", camIndex, routeAddCmd,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	pclose(fp);
}

void multiCastSendSetup() {
	// create a normal UDP socket
	if( (multSendFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
		printf("ERROR   cam %u analyze cannot create UDP socket for sending!, message %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address for data
	memset((char*)&toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(44444); // data port

	printf("INFO    cam %u analyze mulitcast send      IP group address %s port %u\n", camIndex,
			inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));

	// initialize mutex used when sending multicast data
	if( pthread_mutex_init(&txMutex, NULL) != 0 ) {
		printf("ERROR   cam %u analyze multicast send cannot initialize txMutex, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// TODO: add packetCntSyncSend() to synchronize packet counter on x86-64 CPU;
}

void multiCastSendClose() {
	close(multSendFd);
	pthread_mutex_destroy(&txMutex);
}

inline void sendThePacket(char *printString) {
	ssize_t sendAmount = sendto(multSendFd, &txPacket, txPacket.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr));
	if( sendAmount < 0 ) {
		printf("ERROR   cam %u analyze cannot send %s packet, message %s\n", camIndex, printString, strerror(errno));
		printf("WARNING cam %u analyze exit now\n", camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if( sendAmount != txPacket.size ) {
		printf("WARNING cam %u analyze only %zd bytes instead of %u for %s packet\n", camIndex, sendAmount,
				txPacket.size, printString);
		fflush(stdout);
	} else {
		// printf("INFO    cam %u send %4zd bytes for %s packet\n", camIndex, sendAmount, printString);
	}
}

void multiCastReceiveSetup() {
	// create a normal UDP socket
	if( (multReceiveFd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
		printf("ERROR   cam %u analyze cannot create UDP socket for receiving!, message %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port (required when multiple simulation instances are running on x86-64)
	int reuse = 1;
	if( setsockopt(multReceiveFd, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse)) < 0 ) {
		printf("ERROR   cam %u analyze cannot configure port for multiple UDP sockets!, message %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// fill in the local address structure
	struct sockaddr_in localAddr;
	memset((char*)&localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	// on the jetson use the IP address of the Ethernet interface
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY); // accept any incoming messages
	localAddr.sin_port = htons(22222); // configuration port
	// bind to the local address / port
	if( bind(multReceiveFd, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0 ) {
		printf("ERROR   cam %u analyze cannot bind to UDP socket for receiving, %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset(&mreq, 0, sizeof(mreq)); // set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if( setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0 ) {
		printf("ERROR   cam %u analyze cannot join the multicast group for receiving, %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	printf("INFO    cam %u analyze multicast receive   IP group address %s port %u\n", camIndex,
			inet_ntoa(mreq.imr_multiaddr), ntohs(localAddr.sin_port));
}

void multiCastReceiveClose() {
	close(multReceiveFd);
}

// multiCastReceiveLoopBackSetup used to provide data from this application the receive configuration port of this same application (loopback)
void multiCastReceiveLoopBackSetup() {
	// create a normal UDP socket
	if( (multReceiveLoopBackFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
		printf("ERROR   cam %u analyze cannot create UDP socket for sending to local configuration port!, message %s\n",
				camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address for control
	memset((char*)&toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(22222); // configuration port

	printf("INFO    cam %u analyze mulitcast loop back IP group address %s port %u\n", camIndex,
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
	ssize_t sendAmount = sendto(multReceiveLoopBackFd, &lbTxPacket, lbTxPacket.size, 0, (struct sockaddr*)&toAddr,
			sizeof(toAddr));
	if( sendAmount < 0 ) {
		printf("ERROR   cam %u analyze cannot send application exit loop back message, %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if( sendAmount != lbTxPacket.size ) {
		printf("WARNING cam %u analyze only send %zd bytes to loop back instead of %d bytes\n", camIndex, sendAmount,
				lbTxPacket.size);
	}
}

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
	FILE *fp = fopen("/proc/uptime", "r");
	if( fp == NULL ) {
		printf("ERROR   cam %u analyze application cannot open /proc/uptime, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	float cpuUptime = -1.0;
	// CPU uptime in seconds (with fraction)
	if( fscanf(fp, "%f ...", &cpuUptime) <= 0 ) {
		printf("ERROR   cam %u analyze cannot extract the CPU uptime from /proc/uptime, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	fclose(fp);

	float uptime = cpuUptime - analyzeApplicationStartTime;
	if( (uptime >= 0.0) && (uptime <= 65535.0) ) {
		analyzeApplicationUptime = (uint16_t)uptime;
	} else {
		analyzeApplicationUptime = 0xffff;
	}

	pthread_mutex_lock(&txMutex);
	txPacket.pl.u32[0 / 4] = frameCounterPrev;
	txPacket.pl.u16[4 / 2] = analyzeApplicationUptime; // application uptime in seconds
	txPacket.pl.u8[6 / 1] = (valueSum[0] + valueSum[1] + valueSum[2] + valueSum[3]) / (ROI_WIDTH * ROI_HEIGHT); // used to determine if one of the cameras was incorrectly configured (e.g. to dark)

	txPacket.id = camIndex * 64 + 1;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 7;

	sendThePacket((char*)"statics");
	pthread_mutex_unlock(&txMutex);
}

// send the floor long axis line points
inline void linePointsSendLongAxis() {
	// one line point is : xBegin, xEnd, yBegin and yEnd (which are 2 bytes each)
	size_t payloadSize = linePointsIndexLongAxis * sizeof(linePointSt);
	if( payloadSize > CAM_PACKET_PAYLOAD_SIZE ) {
		printf(
				"ERROR   cam %u analyze send %zu long axis line points of %zu bytes do not fit in tx packet payload of %u bytes\n",
				camIndex, linePointsIndexLongAxis, sizeof(linePointSt), CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for( size_t ii = 0; ii < linePointsIndexLongAxis; ii++ ) {
		txPacket.pl.u16[4 * ii] = foundPointsLongAxis[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = foundPointsLongAxis[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = foundPointsLongAxis[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = foundPointsLongAxis[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 2;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char*)"long axis line points");
	pthread_mutex_unlock(&txMutex);
}

// send the floor short axis line points
inline void linePointsSendShortAxis() {
	// one line point is : xBegin, xEnd, yBegin and yEnd (which are 2 bytes each)
	size_t payloadSize = linePointsIndexShortAxis * sizeof(linePointSt);
	if( payloadSize > CAM_PACKET_PAYLOAD_SIZE ) {
		printf(
				"ERROR   cam %u analyze send %zu short axis line points of %zu bytes do not fit in tx packet payload of %u bytes\n",
				camIndex, linePointsIndexShortAxis, sizeof(linePointSt), CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for( size_t ii = 0; ii < linePointsIndexShortAxis; ii++ ) {
		txPacket.pl.u16[4 * ii] = foundPointsShortAxis[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = foundPointsShortAxis[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = foundPointsShortAxis[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = foundPointsShortAxis[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 6;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char*)"short axis line points");
	pthread_mutex_unlock(&txMutex);
}

linePointSt floorPoints[POINTS_MAX_AMOUNT]; // 8187, in normal cases about 1000 line points will be stored
size_t floorPointsIndex = 0; // amount of found floor points

// send the floor points
inline void floorPointsSend() {
	// one floor point is : xBegin, xEnd, yBegin and Yend (which are 2 bytes each)
	size_t payloadSize = floorPointsIndex * sizeof(linePointSt);
	if( payloadSize > CAM_PACKET_PAYLOAD_SIZE ) {
		printf(
				"ERROR   cam %u analyze send %zu floor points of %zu bytes, total %zu bytes, which does not fit in tx buffer of %u bytes\n",
				camIndex, floorPointsIndex, sizeof(linePointSt), payloadSize, CAM_PACKET_PAYLOAD_SIZE);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	pthread_mutex_lock(&txMutex);
	for( size_t ii = 0; ii < floorPointsIndex; ii++ ) {
		txPacket.pl.u16[4 * ii] = floorPoints[ii].xBegin;
		txPacket.pl.u16[4 * ii + 1] = floorPoints[ii].xEnd;
		txPacket.pl.u16[4 * ii + 2] = floorPoints[ii].yBegin;
		txPacket.pl.u16[4 * ii + 3] = floorPoints[ii].yEnd;
	}

	txPacket.id = camIndex * 64 + 5;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + payloadSize;

	sendThePacket((char*)"floor points");
	pthread_mutex_unlock(&txMutex);
	// printf("INFO    cam %u send floor points %zu\n", camIndex, floorPointsIndex);
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

	if( init ) {
		// new line, start with new search
		state = LINE_DET_INIT;
		floorPixels = 0;
		linePixels = 0;
		transferPixels = 0;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if( colorDetection == DET_FLOOR ) {
		// increase the floor pixels and decrease the other pixel counters
		if( floorPixels < lineFloorWindowSize ) {
			floorPixels++;
		}
		if( linePixels > 0 ) {
			linePixels--;
		}
		if( transferPixels > 0 ) {
			transferPixels--;
		}
	} else if( colorDetection == DET_LINE ) {
		// increase the line pixels and decrease the other pixel counters
		if( floorPixels > 0 ) {
			floorPixels--;
		}
		if( linePixels < lineWindowSize ) {
			linePixels++;
		}
		if( transferPixels > 0 ) {
			transferPixels--;
		}
	} else {
		// if it is not a floor pixel or line pixel it is classified as transfer pixel
		// increase the transfer pixel and decrease the other pixel counters
		if( floorPixels > 0 ) {
			floorPixels--;
		}
		if( linePixels > 0 ) {
			linePixels--;
		}
		if( transferPixels < lineTransferPixelsMax ) {
			transferPixels++;
		}
	}

	// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch( state ) {
	case LINE_DET_INIT:
		// we are nowhere, restart the search
		if( colorDetection == DET_FLOOR ) {
			state = LINE_DET_FLOOR_BEFORE;
		}
		break;
	case LINE_DET_FLOOR_BEFORE:
		if( colorDetection == DET_FLOOR ) {
			// floor pixel, keep this state
		} else if( colorDetection == DET_LINE ) {
			// the line is only valid if there are enough floor pixels
			if( floorPixels >= lineFloorPixelsMin ) {
				// enough floor pixels to change state to the line detection state
				state = LINE_DET_ON_LINE;
				xBegin = xx;
			} else {
				if( floorPixels == 0 ) {
					// not enough floor pixels detected, so this cannot be classified as a line, restart search
					state = LINE_DET_INIT;
				}
			}
		} else {
			// not a floor or line pixel, assume this is a transfer pixel
			if( floorPixels >= lineFloorPixelsMin ) {
				// enough floor pixels to change state to the transfer state
				state = LINE_DET_TRANSFER_BEFORE;
			} else {
				if( floorPixels == 0 ) {
					// not enough floor pixels detected, so this cannot be classified as a transfer, restart search
					state = LINE_DET_INIT;
				}
			}
		}
		break;
	case LINE_DET_TRANSFER_BEFORE:
		if( colorDetection == DET_FLOOR ) {
			// it was likely a noise pixel, so we are still on the floor
			state = LINE_DET_FLOOR_BEFORE; // go back one state
		} else if( colorDetection == DET_LINE ) {
			// line pixels found, go to the line detection state
			state = LINE_DET_ON_LINE;
			xBegin = xx;
		} else {
			// no line or floor pixel detected
			if( transferPixels >= lineTransferPixelsMax ) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			}
		}
		break;
	case LINE_DET_ON_LINE:
		if( colorDetection == DET_FLOOR ) {
			// a floor pixel detected, this can be a floor pixel before the line or a floor pixel after the line
			if( linePixels >= linePixelsMin ) {
				state = LINE_DET_FLOOR_AFTER; // proceed two states
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				state = LINE_DET_FLOOR_BEFORE; // go back two states
			}
		} else if( colorDetection == DET_LINE ) {
			// line pixels, keep this state
		} else {
			// no line or floor pixel detected, this can be noise, the transfer area before the line or transfer area after the line
			if( linePixels == 0 ) {
				// not a lot of line pixels, go back to the transfer area before the line
				state = LINE_DET_TRANSFER_BEFORE; // go back one state
			} else if( linePixels >= linePixelsMin ) {
				// enough line pixels, go to the transfer area after the line
				state = LINE_DET_TRANSFER_AFTER; // proceed one state
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				// likely a noise pixel, keep this state
			}
		}
		break;
	case LINE_DET_TRANSFER_AFTER:
		if( colorDetection == DET_FLOOR ) {
			// reached the floor after the line
			state = LINE_DET_FLOOR_AFTER; // proceed one state
		} else if( colorDetection == DET_LINE ) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back one state
		} else {
			// no line or floor pixel detected, check if the transfer area is not to wide
			if( transferPixels >= lineTransferPixelsMax ) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			} else {
				// keep this state, and wait until we reach the floor or determine we are still on the line
			}
		}
		break;

		break;
	case LINE_DET_FLOOR_AFTER:
		if( colorDetection == DET_FLOOR ) {
			// floor pixel, check if we are done
			if( floorPixels >= lineFloorPixelsMin ) {
				// store the line pixel
				if( (xBegin > xEnd) && xEndCheckEnable ) {
					printf("ERROR   cam %u x analyze begin line pixel %u larger then x end line pixel %u for line %u\n",
							camIndex, xBegin, xEnd, yy);
					xEndCheckEnable = false; // TODO: investigate why this happens!, likely get worse when thresholds are set to zero
				} else {
					foundPointsLongAxis[linePointsIndexLongAxis].xBegin = xBegin;
					foundPointsLongAxis[linePointsIndexLongAxis].xEnd = xEnd;
					foundPointsLongAxis[linePointsIndexLongAxis].yBegin = yy;
					foundPointsLongAxis[linePointsIndexLongAxis].yEnd = yy;
					if( linePointsIndexLongAxis < POINTS_MAX_AMOUNT ) {
						linePointsIndexLongAxis++; // prevent writing out of the buffer
					}
				}

				state = LINE_DET_INIT; // restart search for next line
			} // else keep this state
		} else if( colorDetection == DET_LINE ) {
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

	if( init ) {
		// new line, start with new search
		state = LINE_DET_INIT;
		floorPixels = 0;
		linePixels = 0;
		transferPixels = 0;
	}

	// update the pixel counters, which are used by the state machine to determine next state
	if( colorDetection == DET_FLOOR ) {
		// increase the floor pixels and decrease the other pixel counters
		if( floorPixels < lineFloorWindowSize ) {
			floorPixels++;
		}
		if( linePixels > 0 ) {
			linePixels--;
		}
		if( transferPixels > 0 ) {
			transferPixels--;
		}
	} else if( colorDetection == DET_LINE ) {
		// increase the line pixels and decrease the other pixel counters
		if( floorPixels > 0 ) {
			floorPixels--;
		}
		if( linePixels < lineWindowSize ) {
			linePixels++;
		}
		if( transferPixels > 0 ) {
			transferPixels--;
		}
	} else {
		// if it is not a floor pixel or line pixel it is classified as transfer pixel
		// increase the transfer pixel and decrease the other pixel counters
		if( floorPixels > 0 ) {
			floorPixels--;
		}
		if( linePixels > 0 ) {
			linePixels--;
		}
		if( transferPixels < lineTransferPixelsMax ) {
			transferPixels++;
		}
	}

	// use the floor, line and transfer pixel counters together with the current pixel color to step through the states to finally decide if it was a valid line
	switch( state ) {
	case LINE_DET_INIT:
		// we are nowhere, restart the search
		if( colorDetection == DET_FLOOR ) {
			state = LINE_DET_FLOOR_BEFORE;
		}
		break;
	case LINE_DET_FLOOR_BEFORE:
		if( colorDetection == DET_FLOOR ) {
			// floor pixel, keep this state
		} else if( colorDetection == DET_LINE ) {
			// the line is only valid if there are enough floor pixels
			if( floorPixels >= lineFloorPixelsMin ) {
				// enough floor pixels to change state to the line detection state
				state = LINE_DET_ON_LINE;
				xBegin = xx;
			} else {
				if( floorPixels == 0 ) {
					// not enough floor pixels detected, so this cannot be classified as a line, restart search
					state = LINE_DET_INIT;
				}
			}
		} else {
			// not a floor or line pixel, assume this is a transfer pixel
			if( floorPixels >= lineFloorPixelsMin ) {
				// enough floor pixels to change state to the transfer state
				state = LINE_DET_TRANSFER_BEFORE;
			} else {
				if( floorPixels == 0 ) {
					// not enough floor pixels detected, so this cannot be classified as a transfer, restart search
					state = LINE_DET_INIT;
				}
			}
		}
		break;
	case LINE_DET_TRANSFER_BEFORE:
		if( colorDetection == DET_FLOOR ) {
			// it was likely a noise pixel, so we are still on the floor
			state = LINE_DET_FLOOR_BEFORE; // go back one state
		} else if( colorDetection == DET_LINE ) {
			// line pixels found, go to the line detection state
			state = LINE_DET_ON_LINE;
			xBegin = xx;
		} else {
			// no line or floor pixel detected
			if( transferPixels >= lineTransferPixelsMax ) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			}
		}
		break;
	case LINE_DET_ON_LINE:
		if( colorDetection == DET_FLOOR ) {
			// a floor pixel detected, this can be a floor pixel before the line or a floor pixel after the line
			if( linePixels >= linePixelsMin ) {
				state = LINE_DET_FLOOR_AFTER; // proceed two states
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				state = LINE_DET_FLOOR_BEFORE; // go back two states
			}
		} else if( colorDetection == DET_LINE ) {
			// line pixels, keep this state
		} else {
			// no line or floor pixel detected, this can be noise, the transfer area before the line or transfer area after the line
			if( linePixels == 0 ) {
				// not a lot of line pixels, go back to the transfer area before the line
				state = LINE_DET_TRANSFER_BEFORE; // go back one state
			} else if( linePixels >= linePixelsMin ) {
				// enough line pixels, go to the transfer area after the line
				state = LINE_DET_TRANSFER_AFTER; // proceed one state
				xEnd = xx - 1; // the pixel previous was the last valid line pixel
			} else {
				// likely a noise pixel, keep this state
			}
		}
		break;
	case LINE_DET_TRANSFER_AFTER:
		if( colorDetection == DET_FLOOR ) {
			// reached the floor after the line
			state = LINE_DET_FLOOR_AFTER; // proceed one state
		} else if( colorDetection == DET_LINE ) {
			// likely it was a noise pixel and we are still on the line
			state = LINE_DET_ON_LINE; // go back one state
		} else {
			// no line or floor pixel detected, check if the transfer area is not to wide
			if( transferPixels >= lineTransferPixelsMax ) {
				// the space between the floor and line is to large, restart the search
				state = LINE_DET_INIT;
			} else {
				// keep this state, and wait until we reach the floor or determine we are still on the line
			}
		}
		break;

		break;
	case LINE_DET_FLOOR_AFTER:
		if( colorDetection == DET_FLOOR ) {
			// floor pixel, check if we are done
			if( floorPixels >= lineFloorPixelsMin ) {
				// store the line pixel
				if( (xBegin > xEnd) && xEndCheckEnable ) {
					printf("ERROR   cam %u x analyze begin line pixel %u larger then x end line pixel %u for line %u\n",
							camIndex, xBegin, xEnd, yy);
					xEndCheckEnable = false; // TODO: investigate why this happens!, likely get worse when thresholds are set to zero
				} else {
					foundPointsShortAxis[linePointsIndexShortAxis].xBegin = xBegin;
					foundPointsShortAxis[linePointsIndexShortAxis].xEnd = xEnd;
					foundPointsShortAxis[linePointsIndexShortAxis].yBegin = yy;
					foundPointsShortAxis[linePointsIndexShortAxis].yEnd = yy;
					if( linePointsIndexShortAxis < POINTS_MAX_AMOUNT ) {
						linePointsIndexShortAxis++; // prevent writing out of the buffer
					}
				}

				state = LINE_DET_INIT; // restart search for next line
			} // else keep this state
		} else if( colorDetection == DET_LINE ) {
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
	if( xBegin > xEnd ) {
		printf("ERROR   cam %u analyze x begin pixel %u larger then x end pixel %u for floor point %u\n", camIndex,
				xBegin, xEnd, yy);
	} else {
		uint16_t length = xEnd - xBegin + 1;
		if( length >= 10 ) {
			// store floor point
			floorPoints[floorPointsIndex].xBegin = xBegin;
			floorPoints[floorPointsIndex].xEnd = xEnd;
			floorPoints[floorPointsIndex].yBegin = yy;
			floorPoints[floorPointsIndex].yEnd = yy;
			if( floorPointsIndex < POINTS_MAX_AMOUNT ) {
				floorPointsIndex++; // prevent writing out of the buffer
			}
			// printf("INFO    cam %u analyze store floor point %4zu xBegin %3u xEnd %3u y %3u\n", camIndex, floorPointsIndex,
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

	if( xx == 0 ) {
		// new line, start with new search
		foundPixel = false;
	}

	if( foundPixel ) {
		// find out when we did not see a green pixel for a while
		if( colorDetection == DET_FLOOR ) {
			// so we still on the green floor
			lost = 0;
			// update the last known position
			xEnd = xx;
		} else {
			if( lost > lost_max ) {
				// so we did not see a green pixel for a while, so we are not on the green floor anymore
				// figure out if the collected data is worth storing
				storeFloorPoint(xBegin, xEnd, yy);
				foundPixel = false;
			}
			lost++; // we missed the green pixel
		}
	} else {
		// search for green pixel
		if( colorDetection == DET_FLOOR ) {
			foundPixel = true;
			// remember where the green pixels started
			// TODO: add to list
			xBegin = xx;
			xEnd = xx;
			// keep track how long ago we saw a green pixel (we do not want to bail out after the first noise pixel)
			lost = 0;
		}

	}

	if( (xx == (FLOOR_WIDTH - 1)) && (foundPixel) ) {
		// finalize a found pixel in case we are at the end of a line
		storeFloorPoint(xBegin, xEnd, yy);
		foundPixel = false;
		// printf("INFO    cam %u analyze x begin pixel %u end pixel %u yy %u size %u for floor point\n", camIndex, xBegin, xEnd,
		//		yy, size);
	}
}
// run this as 4 different threads
void* jetsonColorDetection(void *arg) {
	int ii = *((int*)arg);
	free(arg);
	size_t bufferIndex = 3 * ii * ROI_WIDTH * ROI_HEIGHT / 4; // use different offset depending on thread
	size_t colorDetectionIndex = ii * ROI_WIDTH * ROI_HEIGHT / 4; // use different offset depending on thread
	size_t lineBegin = ii * ROI_HEIGHT / 4; // each thread is processing 1/4 of the 608 lines
	size_t lineEnd = lineBegin + ROI_HEIGHT / 4 - 1;
	// printf("INFO    cam %u analyze color detection thread %d lines %3zu to %3zu\n", camIndex, ii, lineBegin, lineEnd);

	valueSum[ii] = 0; // start of new frame clear value sum of new frame
	for( size_t yy = lineBegin; yy <= lineEnd - 1; yy++ ) { // lines
		for( size_t xx = 0; xx < ROI_WIDTH; xx++ ) { // pixels
			uint8_t red = shmData[bufferIndex++];
			uint8_t green = shmData[bufferIndex++];
			uint8_t blue = shmData[bufferIndex++];

			// determine maximal value for red, green and blue
			uint32_t value = red;
			uint8_t value_type = 0;
			if( green > value ) {
				value = green;
				value_type = 1; // green
			}
			if( blue > value ) {
				value = blue;
				value_type = 2; // blue
			}

			// determine minimal value for red, green and blue
			uint32_t vmin = red;
			if( blue < vmin ) {
				vmin = blue;
			}
			if( green < vmin ) {
				vmin = green;
			}

			// determine the maximal color difference
			uint32_t vdelta = value - vmin;

			// determine saturation
			uint32_t saturation;
			if( value == 0 ) {
				saturation = 0;
			} else {
				saturation = (255 * vdelta) / value;
			}

			// determine hue
			// 0 to 360 degrees is represented as 0 to 180 degrees*2 so it fits in uint8_t
			int hue;
			if( vdelta == 0 ) {
				hue = 0;
			} else if( value_type == 0 ) {
				// cast to int32_t because green - blue can be negative and negative / uint32_t will cast the negative value to a +429496xxx
				hue = (30 * ((int32_t)green - (int32_t)blue)) / (int32_t)vdelta; // if Vmax = R,  -60 to 60 degrees -> -30 to 30 degrees*2
			} else if( value_type == 1 ) {
				// green
				hue = 60 + (30 * ((int32_t)blue - (int32_t)red)) / (int32_t)vdelta; // if Vmax = G,  60 to 180 degrees -> 30 to 90 degrees*2
			} else {
				// blue
				hue = 120 + (30 * ((int32_t)red - (int32_t)green)) / (int32_t)vdelta; // if Vmax = B, ( 180 to 300 degrees -> 80 to 150 degrees*2
			}
			if( hue < 0 ) {
				hue += 180;
			}
			valueSum[ii] += value;

			colorDetectionBuffer[colorDetectionIndex] = DET_DEFAULT;
			if( (value >= floorValMin) && (saturation >= floorSatMin) && (hue >= (int)floorHueMin)
					&& (hue <= (int)floorHueMax) ) {
				colorDetectionBuffer[colorDetectionIndex] = DET_FLOOR;
			} else if( (value >= lineValMin) && (saturation <= lineSatMax) ) {
				colorDetectionBuffer[colorDetectionIndex] = DET_LINE;
			}

			colorDetectionIndex++;
		}
	}
	return NULL;
}

void* jetsonFloorDetection(void *arg) {
	(void)arg;
	size_t colorDetectionIndex = 0;
	floorPointsIndex = 0; // start with an empty floor point list
	for( size_t yy = 0; yy < ROI_HEIGHT - 1; yy++ ) { // lines (short camera axis)
		// floor pixels do not need to be detected in the air, only scan the lower part of the camera
		for( size_t xx = 0; xx < FLOOR_WIDTH; xx++ ) { // pixels (long camera axis)
			floorDetection(colorDetectionBuffer[colorDetectionIndex], xx, yy);
			colorDetectionIndex++;
		}
		// correct the index for the skipped part at the end of the line (400 to 800)
		colorDetectionIndex += ROI_WIDTH - FLOOR_WIDTH;
	}
	floorPointsSend();
	return NULL;
}

void* jetsonLineDetectionLongAxis(void *arg) {
	(void)arg;
	linePointsIndexLongAxis = 0; // start with an empty line point list
	size_t colorDetectionIndex = 0;

	//printf("INFO    cam %u analyze line detection long camera axis of one frame\n", camIndex);

	// "vertical" scanning (long axis of the camera)
	// scan in the normal (left to right) direction (pixels on one line)
	// Note: the camera is 90 degrees rotated on the robot
	colorDetectionIndex = 0;
	for( size_t yy = 0; yy < ROI_HEIGHT - 1; yy++ ) { // lines (short camera axis)
		// line points do not need to be detected in the air, only scan the lower part of the camera
		for( size_t xx = 0; xx < FLOOR_WIDTH; xx++ ) { // pixels (long camera axis)
			// do not scan on every camera line for a field line to prevent that the ones scanned in the y direction
			// are suppressed (there is a limit on the amount of line points used by the solver)
			if( (yy % 16) == 8 ) {
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

void* jetsonLineDetectionShortAxis(void *arg) {
	(void)arg;
	linePointsIndexShortAxis = 0; // start with an empty line point list
	size_t colorDetectionIndex = 0;

	//printf("INFO    cam %u analyze line detection short camera axis of one frame\n", camIndex);

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

	for( uint16_t xx = 20; xx < 350; xx += 20 ) { // pixels : full range is 0:799
		// process one line from top to bottom
		// scan next vertical line
		colorDetectionIndex = xx; // reset pointer to pixel xx on the first line
		for( uint16_t yy = 0; yy < ROI_HEIGHT - 1; yy++ ) { // lines : 0:607 range
			lineDetectionShortAxis(yy == 0, colorDetectionBuffer[colorDetectionIndex], yy, xx); // swapped xx and yy to be able to re-use the lineDetection function, corrected after this loop
			colorDetectionIndex += ROI_WIDTH; // + 800
		}
	}
	// done with getting the line points in the "horizontal" (top to bottom) direction

	// the lineDetection function is designed for "vertical" (left to right) direction, but used
	// for the "horizontal" (top to bottom) direction, correct the orientation of the found
	// points by swapping the x and y in the list
	for( size_t ii = 0; ii < linePointsIndexShortAxis; ii++ ) {
		// swap xx and yy because we scanned in the other direction
		// Note: because we are scanning on the x-axis the x begin and x end are the same
		uint16_t tmp = foundPointsShortAxis[ii].xBegin;
		foundPointsShortAxis[ii].xBegin = foundPointsShortAxis[ii].yBegin;
		foundPointsShortAxis[ii].yBegin = tmp;
		tmp = foundPointsShortAxis[ii].xEnd;
		foundPointsShortAxis[ii].xEnd = foundPointsShortAxis[ii].yEnd;
		foundPointsShortAxis[ii].yEnd = tmp;
		int width = foundPointsShortAxis[ii].yEnd - foundPointsShortAxis[ii].yBegin;
		if( width < 0 ) {
			printf("ERROR   cam %u analyze line points short axis y width is is %d but should be 0 or larger\n",
					camIndex, width);
		}
	}
	linePointsSendShortAxis();
	return NULL;
}

void* jetsonStatistics(void *arg) {
	(void)arg;
	statisticsSend();
	return NULL;
}

void* jetsonConfig(void *arg) {
	(void)arg;
	while( keepRunning ) {
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		packetT rxPacket;
		// block until data received
		ssize_t nBytes = recvfrom(multReceiveFd, &rxPacket, sizeof(rxPacket), 0, (struct sockaddr*)&fromAddr,
				&fromAddrlen);
		if( nBytes < 0 ) {
			printf("ERROR   cam %u analyze reading from socket, %s", camIndex, strerror(errno));
			fflush(stdout);
			close(multReceiveFd);
			exit(EXIT_FAILURE);
		}

		bool valid = true;
		if( rxPacket.size != nBytes ) {
			printf("ERROR   cam %u analyze packet incomplete, received %zd bytes, but expected %u bytes\n", camIndex,
					nBytes, rxPacket.size);
			fflush(stdout);
			valid = false;
		} else {
			//printf("INFO    received %u bytes from sender %s:%u\n", rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//		ntohs(fromAddr.sin_port));
		}

		if( valid ) {
			if( rxPacket.id == 31 ) {
				size_t expected = 76;
				if( rxPacket.size != expected ) {
					printf("ERROR   cam %u analyze configuration packet should be %zu bytes, but received %u bytes\n",
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

					ii = ii + 14; // skip the config data for ball detection

					floorValMin = rxPacket.pl.u16[ii++];
					floorSatMin = rxPacket.pl.u16[ii++];
					floorHueMin = rxPacket.pl.u16[ii++];
					floorHueMax = rxPacket.pl.u16[ii++];
				}
			} else if( rxPacket.id == 34 ) {
				// analyze process control
				size_t expected = 6;
				if( rxPacket.size != expected ) {
					printf("ERROR   cam %u analyze control packet should be %zu bytes, but received %u bytes\n",
							camIndex, expected, rxPacket.size);
					fflush(stdout);
				}
			} else if( rxPacket.id == 32 ) {
				// packet for grabber process
			} else if( rxPacket.id == 64 ) {
				// packet for system process
			} else if( rxPacket.id == 67 ) {
				// packet for system process pixels offset
			} else if( rxPacket.id == 68 ) {
				// packet for to exit jetson system process
			} else if( rxPacket.id == 69 ) {
				// packet to synchronize frame counter of cam1, cam2 and cam3 with frame counter of cam0
			} else if( rxPacket.id == 98 ) {
				if( rxPacket.pl.u32[0] == 0xdead0011 ) {
					printf("WARNING cam %u analyze received poweroff\n", camIndex);
					fflush(stdout);
					if( system("sudo /sbin/poweroff") <= 0 ) {
						printf("ERROR   cam %u analyze cannot perform \"sudo /sbin/poweroff\", message: %s\n", camIndex,
								strerror(errno));
						fflush(stdout);
						exit(EXIT_FAILURE);
					}
				}
			} else if( rxPacket.id == 99 ) {
				if( rxPacket.pl.u32[0] == 0xdead0022 ) {
					printf("WARNING cam %u analyze received reboot\n", camIndex);
					fflush(stdout);
					if( system("sudo /sbin/reboot") <= 0 ) {
						printf("ERROR   cam %u analyze cannot perform \"sudo /sbin/reboot\", message: %s\n", camIndex,
								strerror(errno));
						fflush(stdout);
						exit(EXIT_FAILURE);
					}
				}
			} else if( rxPacket.id == 100 ) {
				if( rxPacket.pl.u32[0] == 0xdead0033 ) {
					printf("WARNING cam %u analyze received exit application\n", camIndex);
					fflush(stdout);
					keepRunning = false;
				}
			} else {
				printf("ERROR   cam %u analyze received invalid packet id %u\n", camIndex, rxPacket.id);
			} // if (rx.packet.id
		} // if (valid)
	} // while (true)

	return NULL;
}

static inline void processOneFrame() {
	// printf("INFO    cam %u analyze process one frame\n", camIndex);

	for( size_t ii = 0; ii < 4; ii++ ) {
		int *arg2 = (int*)malloc(sizeof(*arg2)); // the free is performed in the jetsonColorDetection function
		if( arg2 == NULL ) {
			printf("ERROR   cam %u analyze cannot allocate memory for thread argument\n", camIndex);
			exit(EXIT_FAILURE);
		}

		*arg2 = ii;

		int err = pthread_create(&(jetsonColorDetectionThreadId[ii]), NULL, &jetsonColorDetection, (void*)arg2);
		if( err != 0 ) {
			printf("ERROR   cam %u analyze cannot start jetsonColorDetection thread, %s\n", camIndex, strerror(err));
		} // if (err
	} // for

	for( size_t ii = 0; ii < 4; ii++ ) {
		pthread_join(jetsonColorDetectionThreadId[ii], NULL);
	}

	int err = pthread_create(&(jetsonStatisticsThreadId), NULL, &jetsonStatistics, NULL);
	if( err != 0 ) {
		printf("ERROR   cam %u analyze cannot start statistics thread, %s\n", camIndex, strerror(err));
	}

	err = pthread_create(&(jetsonFloorDetectionThreadId), NULL, &jetsonFloorDetection, NULL);
	if( err != 0 ) {
		printf("ERROR   cam %u analyze cannot start floor detection thread, %s\n", camIndex, strerror(err));
	}

	err = pthread_create(&(jetsonLineDetectionLongAxisThreadId), NULL, &jetsonLineDetectionLongAxis, NULL);
	if( err != 0 ) {
		printf("ERROR   cam %u analyze cannot start long axis line detection thread, %s\n", camIndex, strerror(err));
	}

	err = pthread_create(&(jetsonLineDetectionShortAxisThreadId), NULL, &jetsonLineDetectionShortAxis, NULL);
	if( err != 0 ) {
		printf("ERROR   cam %u analyze cannot start short axis line detection thread, %s\n", camIndex, strerror(err));
	}

	pthread_join(jetsonLineDetectionLongAxisThreadId, NULL);
	pthread_join(jetsonLineDetectionShortAxisThreadId, NULL);
	pthread_join(jetsonFloorDetectionThreadId, NULL);
	pthread_join(jetsonStatisticsThreadId, NULL);
}

int main(int argc, char **argv) {

	int opt = 0;
	int runTime = -1;

	while( (opt = getopt(argc, argv, "hl:t:")) != -1 ) {
		switch( opt ) {
		case 'h':
			printf("INFO    -l camera index\n");
			printf("         -t run time in seconds (optional)\n");
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
	printf("INFO    cam %u analyze started\n", camIndex);

	// get process id which is used to determine jetson analyze application uptime
	processId = getpid();

	if( processId == 0 ) {
		printf("ERROR   cam %u analyze application process ID cannot be %d\n", camIndex, processId);
		fflush(stdout);
		exit( EXIT_FAILURE);
	} else {
		printf("INFO    cam %u analyze application process ID %d\n", camIndex, processId);
	}

	// get the time the application was started (since CPU is up and running) (to determine application uptime)
	char proc_pid_stat[64];
	sprintf(proc_pid_stat, "/proc/%d/stat", processId);

	FILE *fp = NULL;
	fp = fopen(proc_pid_stat, "r");  // r for read
	if( fp == NULL ) {
		printf("ERROR   cam %u analyze application %s when opening file %s for reading\n", camIndex, strerror(errno),
				proc_pid_stat);
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

	if( retVal <= 0 ) {
		printf("ERROR   cam %u analyze extract values from /proc/%d/stat, message: %s\n", camIndex, processId,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// convert from clock ticks to seconds
	analyzeApplicationStartTime = 1.0 * startTime / sysconf(_SC_CLK_TCK);
	fclose(fp);

	multiCastAddRoute();
	multiCastSendSetup();
	// TODO: send message to x86-64 to indicate analyze process just started
	multiCastReceiveSetup();

	int suppressCounter = 0;
	int shmFd = shm_open("/jetsonGrab", O_RDONLY, 0666);
	if( shmFd < 0 ) {
		printf("ERROR   cam %u analyze cannot open /dev/shm/jetsonGrab with shm_open for reading, message %s\n",
				camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

	shmData = (uint8_t*)mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shmFd, 0);
	printf("INFO    cam %u analyze shared memory mapped to address: %p\n", camIndex, shmData);

	shmDataSend = (uint8_t*)malloc(SHM_SIZE); // for double buffering

	int err = pthread_create(&(jetsonConfigThreadId), NULL, &jetsonConfig, NULL);
	if( err != 0 ) {
		printf("ERROR   cam %u analyze cannot start jetsonConfig thread, %s\n", camIndex, strerror(err));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// flush all normal startup messages to the log file
	fflush(stdout);

	while( keepRunning ) {
		// determine if there is new data in the shared memory buffer
		uint32_t *frameCounterPtr = (uint32_t*)shmData;
		if( *frameCounterPtr != frameCounterPrev ) {
			printf("frame counter %u\n", *frameCounterPtr);
			if( (*frameCounterPtr != (frameCounterPrev + 1)) && (frameCounterPrev != 0) ) {
				if( suppressCounter == 0 ) {
					printf("WARNING cam %u analyze received frame counter %u but expected %u, suppress next 1000\n",
							camIndex, *frameCounterPtr, frameCounterPrev + 1);
					fflush(stdout);
					suppressCounter = 1000;
				}
				suppressCounter--;
			}
			frameCounterPrev = *frameCounterPtr;

			processOneFrame();
		} else {
			// wait and poll again
			usleep(1); // values allowed up to 25 ms (40 fps = 25 ms)
		} // if (*frameCounterPtr != frameCounterPrev) {
		if( runTime == 0 ) {
			keepRunning = false;
		}
		if( runTime > 0 ) {
#define FPS 10
			if( (frameCounterPrev % FPS) == 0 ) {
				runTime--; // decrease every second
			}
		}
	} // while (keepRunning)

	munmap(shmData, SHM_SIZE);
	free(shmDataSend);
	close(shmFd);

	// gracefully shutdown for valgrind report
	// nothing is send anymore to the CPU box
	multiCastSendClose();

	// send a stop command to the still running configuration receive thread
	// graceful shutdown for valgrind
	printf("INFO    cam %u analyze send stop to configuration receive thread\n", camIndex);
	multiCastReceiveLoopBackSetup();
	multiCastReceiveLoopBackStop();
	multiCastReceiveLoopBackClose();

	// stop the configuration thread
	// wait until the jetson configuration thread has been stopped
	pthread_join(jetsonConfigThreadId, NULL);

	// now we also can close the receive configuration port
	multiCastReceiveClose();

	printf("INFO    cam %u analyze all done\n", camIndex);
	return EXIT_SUCCESS;
}
