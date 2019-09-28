 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <arpa/inet.h>
#include <ctime>
#include <errno.h>
#include <iostream>
#include <net/if.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include "multicastSend.hpp"

#ifndef NOROS
#include "FalconsRtDB2.hpp" // for the multiCamStatistics type(s)
#endif

using namespace std;

multicastSend::multicastSend(ballDetection *ballDet[4], ballDetection *ballFarDet[4], cameraReceive *camAnaRecv,
		camSysReceive *camSysRecv, configurator *conf, ballDetection *cyanDet, determinePosition *detPos,
		linePointDetection *linePoint, localization *loc, ballDetection *magentaDet, ballDetection *obstDet[4],
		preprocessor *prep, robotFloor *rFloor) {
	this->ballDet[0] = ballDet[0];
	this->ballDet[1] = ballDet[1];
	this->ballDet[2] = ballDet[2];
	this->ballDet[3] = ballDet[3];
	this->ballFarDet[0] = ballFarDet[0];
	this->ballFarDet[1] = ballFarDet[1];
	this->ballFarDet[2] = ballFarDet[2];
	this->ballFarDet[3] = ballFarDet[3];
	this->camAnaRecv = camAnaRecv;
	this->camSysRecv = camSysRecv;
	this->conf = conf;
	this->cyanDet = cyanDet;
	this->detPos = detPos;
	this->linePoint = linePoint;
	this->loc = loc;
	this->magentaDet = magentaDet;
	this->obstDet[0] = obstDet[0];
	this->obstDet[1] = obstDet[1];
	this->obstDet[2] = obstDet[2];
	this->obstDet[3] = obstDet[3];
	this->prep = prep;
	this->rFloor = rFloor;

	packetCnt = 0;
	packetSize = 0;

	size_t ii = 0;
	char buff[16];

	// create a normal UDP socket
	if ((fd = socket( AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("ERROR     : cannot create UDP socket!, message");
		exit(EXIT_FAILURE);
	}

	// determine if one of the wlan or eth interfaces is running, if not select the lo interface
	bool searchInterface = true;
	struct ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));

	// first check if we can find the Ubuntu 16.04 network interface names
	// TODO: determine interface names with a dynamic mechanism
	sprintf(buff, "wlp2s0"); // wlan
	strcpy(ifr.ifr_name, buff);
	if (ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0) {
		printNetworkInterface(buff, ifr.ifr_flags);
		if (ifr.ifr_flags & IFF_RUNNING) {
			searchInterface = false;
		}
	}

	sprintf(buff, "enp6s0"); // copper
	strcpy(ifr.ifr_name, buff);
	if ((searchInterface == true) && (ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0)) {
		printNetworkInterface(buff, ifr.ifr_flags);
		if (ifr.ifr_flags & IFF_RUNNING) {
			searchInterface = false;
		}
	}

	// now try to find Ubuntu 14.04 network interface names
	ii = 0;
	while (searchInterface && (ii < 20)) {
		sprintf(buff, "wlan%zu", ii);
		strcpy(ifr.ifr_name, buff);
		if (ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0) {
			printNetworkInterface(buff, ifr.ifr_flags);
			if (ifr.ifr_flags & IFF_RUNNING) {
				searchInterface = false;
			}
		}
		ii++;
	}

	ii = 0;
	while (searchInterface && (ii < 20)) {
		sprintf(buff, "eth%zu", ii);
		strcpy(ifr.ifr_name, buff);
		if (ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0) {
			printNetworkInterface(buff, ifr.ifr_flags);
			if (ifr.ifr_flags & IFF_RUNNING) {
				searchInterface = false;
			}
		}
		ii++;
	}

	sprintf(buff, "enx000acd2d1ae8");
	strcpy(ifr.ifr_name, buff);
	if (ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0) {
		printNetworkInterface(buff, ifr.ifr_flags);
		if (ifr.ifr_flags & IFF_RUNNING) {
			searchInterface = false;
		}
	}

	if (searchInterface) {
		strcpy(buff, "lo");
		strcpy(ifr.ifr_name, buff);
		ioctl(fd, SIOCGIFFLAGS, &ifr);
		printNetworkInterface(buff, ifr.ifr_flags);

		// set the interface over which outgoing multicastSend datagrams are sent to lo (because the others are not available)
		// NOTE: this method can also be used if multiple interfaces are available to select which one to use of the active interfaces
		struct ip_mreqn option;
		memset(&option, 0, sizeof(option));
		option.imr_ifindex = if_nametoindex("lo");
		if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF, &option, sizeof(option)) < 0) {
			perror("ERROR     : cannot set the interface for the outgoing multicastSend datagrams, message");
			exit(1);
		}
	}

	// setup the multicast destination address
	printf("INFO      : multicast send ip address %s port %d with frequency %d\n", conf->getMulticast().ip.c_str(),
			conf->getMulticast().port, conf->getMulticast().frequency);
	memset((char *) &toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr(conf->getMulticast().ip.c_str()); // multicastSend group
	toAddr.sin_port = htons(conf->getMulticast().port); // port
}

void multicastSend::send() {
	sendBuf.type = 0x42;
	sendBuf.cnt = packetCnt++;
	sendBuf.size = HEADER_SIZE;

	// sendBuf.size is the number of bytes to send
	ssize_t sendAmount = sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr));
	if (sendAmount < 0) {
		printf("ERROR     : cannot send multicast message, %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	} else if (sendAmount != sendBuf.size) {
		printf("WARNING   : only send %zd bytes instead of %d bytes\n", sendAmount, sendBuf.size);
	}
}

void multicastSend::printNetworkInterface(char *buff, int flags) {
	printf("INFO      : %s flags 0x%04x :", buff, flags);
	if (flags & IFF_UP) {
		printf(" up");
	}
	if (flags & IFF_BROADCAST) {
		printf(", broadcast");
	}
	if (flags & IFF_DEBUG) {
		printf(", debug");
	}
	if (flags & IFF_LOOPBACK) {
		printf(", loopback");
	}
	if (flags & IFF_POINTOPOINT) {
		printf(", point-to-point");
	}
	if (flags & IFF_RUNNING) {
		printf(", running");
	}
	if (flags & IFF_NOARP) {
		printf(", no arp protocol");
	}
	if (flags & IFF_PROMISC) {
		printf(", promiscuous mode");
	}
	if (flags & IFF_NOTRAILERS) {
		printf(", avoid trailers");
	}
	if (flags & IFF_ALLMULTI) {
		printf(", receive all multicast");
	}
	if (flags & IFF_MASTER) {
		printf(", master load balance");
	}
	if (flags & IFF_SLAVE) {
		printf(", slave load balance");
	}
	if (flags & IFF_MULTICAST) {
		printf(", support multicast");
	}
	if (flags & IFF_PORTSEL) {
		printf(", media type via ifmap");
	}
	if (flags & IFF_AUTOMEDIA) {
		printf(", auto media");
	}
	if (flags & IFF_DYNAMIC) {
		printf(", address lost when down");
	}
	// if( flags & IFF_LOWER_UP ) { printf( ", signals L1"); }
	// if( flags & IFF_DORMANT ) { printf( ", signals Dormant"); }
	// if( flags & IFF_ECHO ) { printf( ", echo"); }

	printf("\n");
}

void multicastSend::stats() {
	sendBuf.type = TYPE_STATS;
	sendBuf.cnt = packetCnt++;

	struct timeval tv;
	gettimeofday(&tv, NULL);
	// TODO use double for localTime (but for now compatible with the omni vision interface)
	uint64_t localTime = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;

	vector<ballSt> cyans = cyanDet->getPositions();
	vector<detPosSt> locations = detPos->getLocList();
	vector<ballSt> magentas = magentaDet->getPositions();

	uint8_t ballAmount = 0;
	for (size_t cam = 0; cam < 4; cam++) {
		vector<ballSt> balls = ballDet[cam]->getPositions();
		for (size_t ii = 0; ii < balls.size(); ii++) {
			// only send the amount of balls on the floor (no flying balls)
			float elevation = balls[ii].elevation;
			if (elevation > M_PI) {
				elevation -= 2 * M_PI;
			}
			if ((elevation < -7.0 * M_PI / 180.0) || (elevation == 0.0)) { // elevation above -7.0 degrees is flying ball
				// ball on floor
				if (balls[ii].size >= conf->getBall(ballType).pixels) {
					// large enough
					if (ballAmount != 255) { // prevent overflow uint8_t
						ballAmount++;
					}
				}
			}
		}
	}

	// add the balls and far away balls together
	for (size_t cam = 0; cam < 4; cam++) {
		vector<ballSt> balls = ballFarDet[cam]->getPositions();
		for (size_t ii = 0; ii < balls.size(); ii++) {
			// only send the amount of far away balls on the floor (no flying balls)
			float elevation = balls[ii].elevation;
			if (elevation > M_PI) {
				elevation -= 2 * M_PI;
			}
			if ((elevation < -7.0 * M_PI / 180.0) || (elevation == 0.0)) { // elevation above -7.0 degrees is flying ball
				// far away ball on floor
				if (balls[ii].size >= conf->getBall(ballFarType).pixels) {
					// large enough
					if (ballAmount != 255) { // prevent overflow uint8_t
						ballAmount++;
					}
				}
			}
		}
	}

	uint8_t cyanAmount = 0;
	for (size_t ii = 0; ii < cyans.size(); ii++) {
		if (cyans[ii].size >= conf->getBall(cyanType).pixels) {
			if (cyanAmount != 255) {
				cyanAmount++;
			}
		}
	}

	uint8_t magentaAmount = 0;
	for (size_t ii = 0; ii < magentas.size(); ii++) {
		if (magentas[ii].size >= conf->getBall(magentaType).pixels) {
			if (magentaAmount != 255) {
				magentaAmount++;
			}
		}
	}

	uint8_t obstacleAmount = 0;
	for (size_t cam = 0; cam < 4; cam++) {
		vector<ballSt> obstacles = obstDet[cam]->getPositions();
		for (uint ii = 0; ii < obstacles.size(); ii++) {
			if (obstacles[ii].size >= conf->getBall(obstacleType).pixels) {
				if (obstacleAmount != 255) {
					obstacleAmount++;
				}
			}
		}
	}

	sendBuf.pl.u8[0 / 1] = ballAmount;
	sendBuf.pl.u8[1 / 1] = cyanAmount;
	sendBuf.pl.u8[2 / 1] = magentaAmount;
	sendBuf.pl.u8[3 / 1] = obstacleAmount;

	// amount of ball pixels seen just before the shooter
	size_t tmp = ballDet[0]->getPossessionPixels();
	uint16_t ballPossessionPixels;
	if (tmp >= 65535) {
		ballPossessionPixels = 65535;
	} else {
		ballPossessionPixels = (uint16_t) tmp;
	}
	sendBuf.pl.u16[4 / 2] = ballPossessionPixels;

	sendBuf.pl.u16[6 / 2] = linePoint->getLinePointsPolar().size(); // total amount of line points found
	// multiply by 10 and 100 to be able to send fractions of seconds and FPS
	sendBuf.pl.u16[8 / 2] = 10.0 * prep->getUptime(); // uptime since application startup
	sendBuf.pl.u16[10 / 2] = 100.0 * prep->getFps(); // frames per second for balls and obstacles
	sendBuf.pl.u16[12 / 2] = 100.0 * loc->getFps(); // frames per second for localization
	sendBuf.pl.u32[16 / 4] = prep->getCount(); // amount of frames since application start
	sendBuf.pl.u32[20 / 4] = loc->getCount(); // amount of localizations performed since application start
	// TODO: change to .pl.d64[3]
	sendBuf.pl.u64[24 / 8] = localTime; // 64 bits time in micro seconds since 1 January 1970

	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		camSystemSt value = camSysRecv->getCamSystem(camIndex);

		// calculate the reboot received age from the moment the reset message was received from the raspi
		struct timeval tv;
		gettimeofday(&tv, NULL);
		double localTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
		double deltaTime = localTime - value.rebootTime;
		uint8_t rebootReceivedAge = 0;
		if (deltaTime < 100) { // fit in 2 decimal digits when printing
			rebootReceivedAge = (uint8_t) deltaTime;
		}

		sendBuf.pl.s8[(32 + camIndex * 16) / 1] = value.cmosTemp;
		sendBuf.pl.u8[(33 + camIndex * 16) / 1] = value.cpuLoad; // warning: fixed point, divide by 32.0 to get correct value
		sendBuf.pl.u8[(34 + camIndex * 16) / 1] = value.cpuTemp;
		sendBuf.pl.u8[(35 + camIndex * 16) / 1] = value.gpuTemp;
		sendBuf.pl.u8[(36 + camIndex * 16) / 1] = value.cpuStatus;
		sendBuf.pl.u8[(37 + camIndex * 16) / 1] = camAnaRecv->getCamValAverage(camIndex);
		sendBuf.pl.u8[(38 + camIndex * 16) / 1] = rebootReceivedAge;
		sendBuf.pl.u8[(39 + camIndex * 16) / 1] = 0; // free
		sendBuf.pl.u16[(40 + camIndex * 16) / 2] = value.sysApplUptime;
		sendBuf.pl.u16[(42 + camIndex * 16) / 2] = value.cpuUptime;
		sendBuf.pl.u16[(44 + camIndex * 16) / 2] = camAnaRecv->getAnaApplUptime(camIndex);
		sendBuf.pl.u16[(46 + camIndex * 16) / 2] = 0; // free
		sendBuf.pl.u32[(48 + camIndex * 16) / 4] = camAnaRecv->getAnaFrameCounter(camIndex);
	}
	sendBuf.size = HEADER_SIZE + 32 + 4 * 20;

	if (sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr)) < 0) {
		perror("ERROR     : cannot send statistics multicast message, check if route exist, message");
		exit(EXIT_FAILURE);
	}

#ifndef NOROS

	// copy the statistics to the rtdb struct
	multiCamStatistics multiCamStats;

	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		camSystemSt value = camSysRecv->getCamSystem(camIndex);

		// calculate the reboot received age from the moment the reset message was received from the raspi
		struct timeval tv;
		gettimeofday(&tv, NULL);
		double localTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
		double deltaTime = localTime - value.rebootTime;
		uint8_t rebootReceivedAge = 0;
		if (deltaTime < 100) { // fit in 2 decimal digits when printing
			rebootReceivedAge = (uint8_t) deltaTime;
		}

		raspiStatSingle raspi;
		raspi.frameCounter = camAnaRecv->getAnaFrameCounter(camIndex);
		raspi.cpuUptime = value.cpuUptime;
		raspi.grabberUptime = 0;
		raspi.systemUptime = value.sysApplUptime;
		raspi.analyzeUptime = camAnaRecv->getAnaApplUptime(camIndex);
		raspi.rebootReceivedAge = rebootReceivedAge;
		raspi.cpuLoad = value.cpuLoad / 32.0; // warning: fixed point, divide by 32.0 to get correct value
		raspi.cpuTemp = value.cpuTemp;
		raspi.gpuTemp = value.gpuTemp;
		raspi.cmosTemp = value.cmosTemp;

		// 0: under-voltage
		// 1: arm frequency capped
		// 2: currently throttled
		// 3: soft temp limit (added in 2018)

		// WARNING: on sender side bits 16-19 have been shifted to bits 4-7
		// 4: under-voltage has occurred
		// 5: arm frequency capped has occurred
		// 6: throttling has occurred
		// 7: soft temp limit (added in 2018)

		raspi.underVoltage = (((value.cpuStatus >> 0) & 0x1) == 1);
		raspi.underVoltageOccurred = (((value.cpuStatus >> 4) & 0x1) == 1);

		raspi.cpuFrequencyCap = (((value.cpuStatus >> 1) & 0x1) == 1);
		raspi.cpuFrequencyCapOccurred = (((value.cpuStatus >> 5) & 0x1) == 1);

		raspi.cpuThrottle = (((value.cpuStatus >> 2) & 0x1) == 1);
		raspi.cpuThrottleOccurred = (((value.cpuStatus >> 6) & 0x1) == 1);

#ifdef NONO
		// TODO: add to rtdb interface
		raspi.cpuSoftTempLimit = (((value.cpuStatus >> 3) & 0x1) == 1);
		raspi.cpuSoftTempOccurred = (((value.cpuStatus >> 7) & 0x1) == 1);
#endif

		multiCamStats.raspi.push_back(raspi);
	}

	multiCamStats.ballAmount = ballAmount;
	multiCamStats.obstacleAmount = obstacleAmount;

	multiCamStats.ballPossessionPixels = ballPossessionPixels;

	multiCamStats.linePoints = linePoint->getLinePointsPolar().size(); // total amount of line points found

	multiCamStats.multiCamUptime = prep->getUptime(); // uptime since application startup
	multiCamStats.multiCamTime = localTime; // 64 bits time in micro seconds since 1 January 1970

	multiCamStats.cam0Fps = prep->getFps(); // frames per second for balls and obstacles;
	multiCamStats.localizationFps = loc->getFps(); // frames per second for localization

	multiCamStats.cam0Frames = prep->getCount(); // amount of frames since application start
	multiCamStats.localizationFrames = loc->getCount(); // amount of localizations performed since application start

	for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); iter++) {
		if (*iter != NULL) {
			(*iter)->update_multi_cam_statistics(multiCamStats);
		}
	}
#endif

}

void multicastSend::locList() {
	vector<detPosSt> locList = detPos->getLocList();

	uint16_t payloadIndex = 0;
	size_t locListSend = locList.size();
	if (locListSend > 3) {
		locListSend = 3;
	} // do not send more then 3 locs

	for (size_t ii = 0; ii < locListSend; ii++) {
		// maximal x floor size in pixels : 22 + xx = 24 meter / 2 cm = 1200 pixels
		// maximal y floor size in pixels : 14 + xx = 16 meter / 2 cm = 800 pixels
		// use fixed point with 5 bits for fraction, range x[0:1199], y[0:799], 2^16/1200=54
		sendBuf.pl.u16[payloadIndex++] = round(32.0 * locList[ii].pos.x);
		sendBuf.pl.u16[payloadIndex++] = round(32.0 * locList[ii].pos.y);
		// use fixed point with 6 bits for fraction, range rz[0:359]
		sendBuf.pl.u16[payloadIndex++] = round(64.0 * locList[ii].pos.rz);
		// use fixed point with 16 bits for fraction, range score[0:1]
		if (locList[ii].score >= 1.0) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(65535.0 * locList[ii].score);
		}
		if (locList[ii].lastActive >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].lastActive;
		}
		if (locList[ii].age >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].age;
		}
		if (locList[ii].amountOnFloor >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].amountOnFloor;
		}
		if (locList[ii].numberOfTries >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].numberOfTries;
		}
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if (payloadIndex > 0) {
		sendBuf.type = TYPE_LOCALIZATION;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if (sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr)) < 0) {
			perror("ERROR     : cannot send localization multicast message, check if route exist, message");
			exit(1);
		}
	}
}

void multicastSend::goodEnoughLoc() {
	detPosSt goodEnoughLoc = detPos->getGoodEnoughLocExport();
	detPosSt goodEnoughLocRos = detPos->getGoodEnoughLocExportRos();

	uint16_t payloadIndex = 0;
	if (goodEnoughLoc.goodEnough) {
		// maximal x floor size in pixels : 22 + xx = 24 meter / 2 cm = 1200 pixels
		// maximal y floor size in pixels : 14 + xx = 16 meter / 2 cm = 800 pixels
		// use fixed point with 5 bits for fraction, range x[0:1199], y[0:799], 2^16/1200=54
		sendBuf.pl.u16[payloadIndex++] = round(32.0 * goodEnoughLoc.pos.x);
		sendBuf.pl.u16[payloadIndex++] = round(32.0 * goodEnoughLoc.pos.y);
		// use fixed point with 6 bits for fraction, range rz[0:359]
		sendBuf.pl.u16[payloadIndex++] = round(64.0 * goodEnoughLoc.pos.rz);

		// maximal x floor size in meters : 22 + xx = 24 meter => [-12:12] meter
		// maximal y floor size in meters : 14 + xx = 16 meter => [-8:8] meter
		// use signed fixed point with 11 bits for fraction, range x[-8.0:8.0], y[-12:12], 2^15/12=2723
		sendBuf.pl.s16[payloadIndex++] = round(2048.0 * goodEnoughLocRos.pos.x);
		sendBuf.pl.s16[payloadIndex++] = round(2048.0 * goodEnoughLocRos.pos.y);
		// use fixed point with 13 bits for fraction, range rz[0:6.28], 2^16/6.28=10436
		sendBuf.pl.u16[payloadIndex++] = round(8192.0 * goodEnoughLocRos.pos.rz);

		// use fixed point with 16 bits for fraction, range score[0:1]
		if (goodEnoughLoc.score >= 1.0) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(65535.0 * goodEnoughLoc.score);
		}
		if (goodEnoughLoc.lastActive >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.lastActive;
		}
		if (goodEnoughLoc.age >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.age;
		}
		if (goodEnoughLoc.amountOnFloor >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.amountOnFloor;
		}
		if (goodEnoughLoc.numberOfTries >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.numberOfTries;
		}
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if (payloadIndex > 0) {
		sendBuf.type = TYPE_GOOD_ENOUGH_LOC;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if (sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr)) < 0) {
			perror("ERROR     : cannot send good enough multicast message, check if route exist, message");
			exit(1);
		}
	}
}

// send the floor line points
void multicastSend::floorLinePoints() {
	vector<angleRadiusSt> floorLinePoints = linePoint->getLinePointsPolar();

	uint16_t payloadIndex = 0;
	size_t sendMaxPoints = floorLinePoints.size();
	if (sendMaxPoints > 50) { // TODO: make configurable
		sendMaxPoints = 50;
	}

	for (size_t ii = 0; ii < sendMaxPoints; ii++) {
		// remove viewer expect radius in meters
		double radiusMeter = 0.001 * floorLinePoints[ii].radius; // radius provided in mm
		// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
		sendBuf.pl.u16[payloadIndex++] = round(2048.0 * radiusMeter);

		// remote viewer expects angle in radians
		double angleRad = floorLinePoints[ii].angle; // angle provided in radians
		angleRad = fmod(angleRad + 2 * M_PI, 2 * M_PI); // be sure the angle is in the 0 to 2 PI range
		// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
		sendBuf.pl.u16[payloadIndex++] = round(8192.0 * angleRad);
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if (payloadIndex > 0) {
		sendBuf.type = TYPE_LINEPOINTS;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if (sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr)) < 0) {
			perror("ERROR     : cannot send floor point list multicast message, check if route exist, message");
			exit(1);
		}
	}
}

// send the ball list, also used for the cyanList, magentaList and obstacles
void multicastSend::ballList(size_t type) {
	vector<ballSt> ballList;
	ballList.empty();
	if (type == TYPE_BALLDETECTION) {
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> tmp = ballDet[cam]->getAndClearPositionsRemoteViewer();
			for (size_t ii = 0; ii < tmp.size(); ii++) {
				tmp[ii].azimuth += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated
				ballList.push_back(tmp[ii]);
			}
		}
	} else if (type == TYPE_BALLFARDETECTION) {
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> tmp = ballFarDet[cam]->getAndClearPositionsRemoteViewer();
			for (size_t ii = 0; ii < tmp.size(); ii++) {
				tmp[ii].azimuth += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated
				ballList.push_back(tmp[ii]);
			}
		}
	} else if (type == TYPE_CYANDETECTION) {
		ballList = cyanDet->getPositionsExport();
	} else if (type == TYPE_MAGENTADETECTION) {
		ballList = magentaDet->getPositionsExport();
	} else if (type == TYPE_OBSTACLES) {
		for (size_t cam = 0; cam < 4; cam++) {
			vector<ballSt> tmp = obstDet[cam]->getPositionsExport();
			for (size_t ii = 0; ii < tmp.size(); ii++) {
				tmp[ii].azimuth += cam * CV_PI / 2.0; // cam * 90 degrees, every camera is 90 degrees rotated
				ballList.push_back(tmp[ii]);
			}
		}
	}

	uint16_t payloadIndex = 0;
	for (size_t ii = 0; ii < ballList.size(); ii++) {
		double ballSize = 10 * ballList[ii].size; // TODO: create better scale factor
		if (ballSize >= 65535) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = ballSize;
		}

		// remote viewer expects radius in meters
		double radiusMeter = 0.001 * ballList[ii].radius; // radius provided in mm
		// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
		sendBuf.pl.u16[payloadIndex++] = round(2048.0 * radiusMeter);

		// remote viewer expects azimuth in radians
		double azimuth = ballList[ii].azimuth; // azimuth provided in radians
		azimuth = fmod(azimuth + 2 * M_PI, 2 * M_PI); // be sure the angle is in the 0 to 2 PI range
		// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
		sendBuf.pl.u16[payloadIndex++] = round(8192.0 * azimuth);
		// remote viewer expects elevation in radians
		double elevation = ballList[ii].elevation; // elevation provided in radians
		elevation = fmod(elevation + 2 * M_PI, 2 * M_PI); // be sure the angle is in the 0 to 2 PI range
		// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
		sendBuf.pl.u16[payloadIndex++] = round(8192.0 * elevation);
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if (payloadIndex > 0) {
		sendBuf.type = type;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2; // each payload value is 2 bytes

		if (sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr)) < 0) {
			perror("ERROR     : cannot send ball, obstacle, .. list multicast message, check if route exist, message");
			exit(1);
		}
	}
}

void multicastSend::attach(observer *observer) {
	vecObservers.push_back(observer);
}

void multicastSend::detach(observer *observer) {
	vecObservers.erase(std::remove(vecObservers.begin(), vecObservers.end(), observer), vecObservers.end());
}

