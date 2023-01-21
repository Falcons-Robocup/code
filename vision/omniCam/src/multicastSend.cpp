// Copyright 2017-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <arpa/inet.h>
#include <ctime>
#include <iostream>
#include <net/if.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "multicastSend.hpp"

using namespace std;

multicastSend::multicastSend(ballDetection *ballDet, ballPossession *ballPos, configurator *conf,
		ballDetection *cyanDet, determinePosition *detPos, linePointDetection *linePoint, localization *loc,
		ballDetection *magentaDet, obstacleDetection *obstDet, preprocessor *prep, robotFloor *rFloor) {
	this->ballDet = ballDet;
	this->ballPos = ballPos;
	this->conf = conf;
	this->cyanDet = cyanDet;
	this->detPos = detPos;
	this->linePoint = linePoint;
	this->loc = loc;
	this->magentaDet = magentaDet;
	this->obstDet = obstDet;
	this->prep = prep;
	this->rFloor = rFloor;

	packetCnt = 0;
	packetSize = 0;

	size_t ii = 0;
	char buff[64];

	// create a normal UDP socket
	if( (fd = socket( AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("ERROR  : cannot create UDP socket!, message");
		exit(EXIT_FAILURE);
	}

	// determine if one of the wlan or eth interfaces is running, if not select the lo interface
	bool searchInterface = true;
	struct ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));

	// first check if we can find the Ubuntu 16.04 network interface names
	// TODO: determine interface names dynamically
	strcpy(ifr.ifr_name, "wlp2s0"); // wlan
	if( ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0 ) {
		printNetworkInterface(buff, ifr.ifr_flags);
		if( ifr.ifr_flags & IFF_RUNNING ) {
			searchInterface = false;
		}
	}

	strcpy(ifr.ifr_name, "enp6s0"); // copper
	if( (searchInterface == true) && (ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0) ) {
		printNetworkInterface(buff, ifr.ifr_flags);
		if( ifr.ifr_flags & IFF_RUNNING ) {
			searchInterface = false;
		}
	}

	// now try to find Ubuntu 14.04 network interface names
	ii = 0;
	while( searchInterface && (ii < 20) ) {
		sprintf(buff, "wlan%zu", ii);
		strcpy(ifr.ifr_name, buff);
		if( ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0 ) {
			printNetworkInterface(buff, ifr.ifr_flags);
			if( ifr.ifr_flags & IFF_RUNNING ) {
				searchInterface = false;
			}
		}
		ii++;
	}

	ii = 0;
	while( searchInterface && (ii < 20) ) {
		sprintf(buff, "eth%zu", ii);
		strcpy(ifr.ifr_name, buff);
		if( ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0 ) {
			printNetworkInterface(buff, ifr.ifr_flags);
			if( ifr.ifr_flags & IFF_RUNNING ) {
				searchInterface = false;
			}
		}
		ii++;
	}

	if( searchInterface ) {
		strcpy(buff, "lo");
		strcpy(ifr.ifr_name, buff);
		ioctl(fd, SIOCGIFFLAGS, &ifr);
		printNetworkInterface(buff, ifr.ifr_flags);

		// set the interface over which outgoing multicastSend datagrams are sent to lo (because the others are not available)
		// NOTE: this method can also be used if multiple interfaces are available to select which one to use of the active interfaces
		struct ip_mreqn option = { 0, 0, 0 };
		option.imr_ifindex = if_nametoindex("lo");
		if( setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF, &option, sizeof(option)) < 0 ) {
			perror("ERROR  : cannot set the interface for the outgoing multicastSend datagrams, message");
			exit(1);
		}
	}

	// setup the multicast destination address
	printf("INFO      : multicast ip address %s port %d with frequency %d\n", conf->getMulticast().ip.c_str(),
			conf->getMulticast().port, conf->getMulticast().frequency);
	memset((char*)&toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr(conf->getMulticast().ip.c_str()); // multicastSend group
	toAddr.sin_port = htons(conf->getMulticast().port); // port
}

void multicastSend::send() {
	sendBuf.type = 0x42;
	sendBuf.cnt = packetCnt++;
	sendBuf.size = HEADER_SIZE;

	// idx is now number of bytes
	if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
		perror("ERROR  : cannot send message, message");
		exit(1);
	}
}

void multicastSend::printNetworkInterface(char *buff, int flags) {
	printf("INFO      : ethernet %s flags 0x%04x :", buff, flags);
	if( flags & IFF_UP ) {
		printf(" up");
	}
	if( flags & IFF_BROADCAST ) {
		printf(", broadcast");
	}
	if( flags & IFF_DEBUG ) {
		printf(", debug");
	}
	if( flags & IFF_LOOPBACK ) {
		printf(", loopback");
	}
	if( flags & IFF_POINTOPOINT ) {
		printf(", point-to-point");
	}
	if( flags & IFF_RUNNING ) {
		printf(", running");
	}
	if( flags & IFF_NOARP ) {
		printf(", no arp protocol");
	}
	if( flags & IFF_PROMISC ) {
		printf(", promiscuous mode");
	}
	if( flags & IFF_NOTRAILERS ) {
		printf(", avoid trailers");
	}
	if( flags & IFF_ALLMULTI ) {
		printf(", receive all multicast");
	}
	if( flags & IFF_MASTER ) {
		printf(", master load balance");
	}
	if( flags & IFF_SLAVE ) {
		printf(", slave load balance");
	}
	if( flags & IFF_MULTICAST ) {
		printf(", support multicast");
	}
	if( flags & IFF_PORTSEL ) {
		printf(", media type via ifmap");
	}
	if( flags & IFF_AUTOMEDIA ) {
		printf(", auto media");
	}
	if( flags & IFF_DYNAMIC ) {
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
	uint64_t localTime = tv.tv_sec * 1000000 + tv.tv_usec;

	vector<ballSt> balls = ballDet->getPositions();
	vector<ballSt> cyans = cyanDet->getPositions();
	vector<detPosSt> locations = detPos->getLocList();
	vector<ballSt> magentas = magentaDet->getPositions();
	vector<obstacleSt> obstacles = obstDet->getPositions();

	uint8_t ballAmount = 0;
	for( size_t ii = 0; ii < balls.size(); ii++ ) {
		if( balls[ii].size >= conf->getBall(ballType).size ) {
			ballAmount++;
		}
	}

	uint8_t cyanAmount = 0;
	for( size_t ii = 0; ii < cyans.size(); ii++ ) {
		if( cyans[ii].size >= conf->getBall(cyanType).size ) {
			cyanAmount++;
		}
	}

	uint8_t magentaAmount = 0;
	for( size_t ii = 0; ii < magentas.size(); ii++ ) {
		if( magentas[ii].size >= conf->getBall(magentaType).size ) {
			magentaAmount++;
		}
	}

	uint8_t obstacleAmount = 0;
	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		if( obstacles[ii].size >= conf->getObstacle().size ) {
			obstacleAmount++;
		}
	}

	sendBuf.pl.u8[0] = ballAmount;
	sendBuf.pl.u8[1] = cyanAmount;
	sendBuf.pl.u8[2] = magentaAmount;
	sendBuf.pl.u8[3] = obstacleAmount;
	if( ballPos->getPossessionPixels() >= 65535 ) {
		sendBuf.pl.u16[2] = 65535;
	} else {
		// amount of ball pixels seen just before the shooter
		sendBuf.pl.u16[2] = ballPos->getPossessionPixels();
	}
	sendBuf.pl.u16[3] = linePoint->getLinePointsRadianMeter().size(); // total amount of line points found
	sendBuf.pl.u16[4] = 10.0 * prep->getUptime(); // uptime since application startup
	sendBuf.pl.u16[5] = 100.0 * prep->getFps(); // frames per second for balls and obstacles
	sendBuf.pl.u16[6] = 100.0 * loc->getFps(); // frames per second for localization
	sendBuf.pl.u32[4] = prep->getCount(); // amount of frames since application start
	sendBuf.pl.u32[5] = loc->getCount(); // amount of localizations performed since application start
	sendBuf.pl.u64[3] = localTime; // 64 bits time in micro seconds since 1 January 1970
	sendBuf.size = HEADER_SIZE + 4 * 8; // update with last send.pl.u64

	if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
		perror("ERROR  : cannot send message, message");
		exit(1);
	}
}

void multicastSend::locList() {
	vector<detPosSt> locList = detPos->getLocList();

	uint16_t payloadIndex = 0;
	size_t locListSend = locList.size();
	if( locListSend > 3 ) {
		locListSend = 3;
	} // do not send more then 3 locs

	for( size_t ii = 0; ii < locListSend; ii++ ) {
		// maximal x floor size in pixels : 22 + xx = 24 meter / 2 cm = 1200 pixels
		// maximal y floor size in pixels : 14 + xx = 16 meter / 2 cm = 800 pixels
		// use fixed point with 5 bits for fraction, range x[0:1199], y[0:799], 2^16/1200=54
		sendBuf.pl.u16[payloadIndex++] = round(32.0 * locList[ii].pos.x);
		sendBuf.pl.u16[payloadIndex++] = round(32.0 * locList[ii].pos.y);
		// use fixed point with 6 bits for fraction, range rz[0:359]
		sendBuf.pl.u16[payloadIndex++] = round(64.0 * locList[ii].pos.rz);
		// use fixed point with 16 bits for fraction, range score[0:1]
		if( locList[ii].score >= 1.0 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(65535.0 * locList[ii].score);
		}
		if( locList[ii].lastActive >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].lastActive;
		}
		if( locList[ii].age >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].age;
		}
		if( locList[ii].amountOnFloor >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].amountOnFloor;
		}
		if( locList[ii].numberOfTries >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = locList[ii].numberOfTries;
		}
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_LOCALIZATION;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

void multicastSend::goodEnoughLoc() {
	detPosSt goodEnoughLoc = detPos->getGoodEnoughLocExport();
	detPosSt goodEnoughLocRos = detPos->getGoodEnoughLocExportRos();

	uint16_t payloadIndex = 0;
	if( goodEnoughLoc.goodEnough ) {
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
		if( goodEnoughLoc.score >= 1.0 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(65535.0 * goodEnoughLoc.score);
		}
		if( goodEnoughLoc.lastActive >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.lastActive;
		}
		if( goodEnoughLoc.age >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.age;
		}
		if( goodEnoughLoc.amountOnFloor >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.amountOnFloor;
		}
		if( goodEnoughLoc.numberOfTries >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = goodEnoughLoc.numberOfTries;
		}
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_GOOD_ENOUGH_LOC;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

// send the floor line points
void multicastSend::floorLinePoints() {
	vector<angleRadiusSt> floorLinePoints = linePoint->getLinePointsRadianMeter();

	uint16_t payloadIndex = 0;
	size_t sendMaxPoints = floorLinePoints.size();
	if( sendMaxPoints > 50 ) {
		sendMaxPoints = 50;
	} // TODO: make configurable
	for( size_t ii = 0; ii < sendMaxPoints; ii++ ) {
		// remove viewer expect radius in meters
		double radiusMeter = 0.02 * floorLinePoints[ii].radius; // radius provided in pixels of 0.02 meter
		// fixed point: 65536/2048 = 32, radius range 0 to 24 meters (for large field)
		if( radiusMeter < 0.0 ) {
			printf("ERROR  : error when sending floor line points with negative radius of %.1f meter\n", radiusMeter);
			sendBuf.pl.u16[payloadIndex++] = 0;
		} else if( (2048.0 * radiusMeter) > 65535.0 ) {
			printf("ERROR  : overflow when sending floor line points of %.1f meters while the max is 32 meters\n",
					radiusMeter);
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(2048.0 * radiusMeter);
		}

		// remote viewer expects angle in radians
		double angleRad = floorLinePoints[ii].angle; // angle provided in radians
		angleRad = fmod(angleRad + 2 * M_PI, 2 * M_PI); // be sure the angle is in the 0 to 2 PI range
		// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
		sendBuf.pl.u16[payloadIndex++] = round(8192.0 * angleRad);
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_LINEPOINTS;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

// send the ball list, also used for the cyanList and magentaList
void multicastSend::ballList(size_t type) {
	vector<ballSt> ballList;
	if( type == TYPE_BALLDETECTION ) {
		ballList = ballDet->getPositionsExport();
	} else if( type == TYPE_CYANDETECTION ) {
		ballList = cyanDet->getPositionsExport();
	} else if( type == TYPE_MAGENTADETECTION ) {
		ballList = magentaDet->getPositionsExport();
	}

	uint16_t payloadIndex = 0;
	for( size_t ii = 0; ii < ballList.size(); ii++ ) {
		if( ballList[ii].size >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = ballList[ii].size;
		}

		// remote viewer expects radius in meters
		double radiusMeter = 0.02 * ballList[ii].radiusLin; // radius provided in pixels of 0.02 meter
		// fixed point: 65536/2048 = 32 meter, radius range 0 to 24 meters (for large field)
		if( radiusMeter < 0.0 ) {
			printf("ERROR  : error when sending ball with negative radius of %.1f meter\n", radiusMeter);
			sendBuf.pl.u16[payloadIndex++] = 0;
		} else if( (2048.0 * radiusMeter) > 65535.0 ) {
			printf("ERROR  : overflow when sending ball radius of %.1f meters while the max is 32 meters\n",
					radiusMeter);
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(2048.0 * radiusMeter);
		}

		// remote viewer expects angle in radians
		double angleDeg = ballList[ii].angle; // angle provided in degrees
		double angleRad = M_PI * angleDeg / 180.0;
		angleRad = fmod(angleRad + 2 * M_PI, 2 * M_PI); // be sure the angle is in the 0 to 2 PI range
		// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
		sendBuf.pl.u16[payloadIndex++] = round(8192.0 * angleRad);
		sendBuf.pl.u16[payloadIndex++] = 0; // not used elevation
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if( payloadIndex > 0 ) {
		sendBuf.type = type;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2; // each payload index is 2 bytes

		if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

// send the obstacle list
void multicastSend::obstaclelList() {
	vector<obstacleSt> obstacleList = obstDet->getPositionsExport();

	uint16_t payloadIndex = 0;
	for( size_t ii = 0; ii < obstacleList.size(); ii++ ) {
		// only send the obstacles that are large enough
		if( obstacleList[ii].size > conf->getObstacle().size ) {
			if( obstacleList[ii].size >= 65535 ) {
				sendBuf.pl.u16[payloadIndex++] = 65535;
			} else {
				sendBuf.pl.u16[payloadIndex++] = obstacleList[ii].size;
			}
			// remote viewer expects radius in meters
			double radiusMeter = 0.02 * obstacleList[ii].radiusLin; // radius provided in pixels of 0.02 meter
			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			sendBuf.pl.u16[payloadIndex++] = round(2048.0 * radiusMeter);

			// remote viewer expects angle in radians
			double angleDeg = obstacleList[ii].angle; // angle provided in degrees
			double angleRad = M_PI * angleDeg / 180.0;
			angleRad = fmod(angleRad + 2 * M_PI, 2 * M_PI); // be sure the angle is in the 0 to 2 PI range
			// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
			sendBuf.pl.u16[payloadIndex++] = round(8192.0 * angleRad);
			sendBuf.pl.u16[payloadIndex++] = 0; // not used elevation
		}
	}

	// not required to send the package, if nothing has been stored in the payload (previous lines)
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_OBSTACLES;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2; // each payload index is 2 bytes

		if( sendto(fd, &sendBuf, sendBuf.size, 0, (struct sockaddr*)&toAddr, sizeof(toAddr)) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}
