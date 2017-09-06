 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2017 Andre Pool
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

multicastSend::multicastSend(ballDetection *ballDet,
		ballPossession *ballPos,
		configurator *conf,
		ballDetection *cyanDet,
		determinePosition *detPos,
		linePointDetection *linePoint,
		localization *loc,
		ballDetection *magentaDet,
		obstacleDetection *obstDet,
		preprocessor *prep,
		robotFloor *rFloor) {
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
	char buff[16];

	// create a normal UDP socket
	if ( ( fd = socket( AF_INET, SOCK_DGRAM, 0 ) ) < 0 ) {
		perror("ERROR  : cannot create UDP socket!, message");
		exit(EXIT_FAILURE);
	}

	// determine if one of the wlan or eth interfaces is running, if not select the lo interface
	bool searchInterface = true;
	struct ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));

	ii = 0;
	while( searchInterface && ( ii < 20 ) ) {
		sprintf( buff, "wlan%zu", ii );
		strcpy(ifr.ifr_name, buff);
		if( ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0 ) {
			printNetworkInterface( buff, ifr.ifr_flags );
			if( ifr.ifr_flags & IFF_RUNNING ) { searchInterface = false; }
		}
		ii++;
	}

	ii = 0;
	while( searchInterface && ( ii < 20 ) ) {
		sprintf( buff, "eth%zu", ii );
		strcpy(ifr.ifr_name, buff);
		if( ioctl(fd, SIOCGIFFLAGS, &ifr) >= 0 ) {
			printNetworkInterface( buff, ifr.ifr_flags );
			if( ifr.ifr_flags & IFF_RUNNING ) { searchInterface = false; }
		}
		ii++;
	}

	if( searchInterface ) {
		strcpy(buff, "lo");
		strcpy(ifr.ifr_name, buff);
		ioctl(fd, SIOCGIFFLAGS, &ifr);
		printNetworkInterface( buff, ifr.ifr_flags );

		// set the interface over which outgoing multicastSend datagrams are sent to lo (because the others are not available)
		// NOTE: this method can also be used if multiple interfaces are available to select which one to use of the active interfaces
		struct ip_mreqn option = { 0, 0, 0 };
		option.imr_ifindex = if_nametoindex("lo");
		if( setsockopt( fd, IPPROTO_IP, IP_MULTICAST_IF, &option, sizeof(option)) < 0 ) {
			perror("ERROR  : cannot set the interface for the outgoing multicastSend datagrams, message");
			exit(1);
		}
	}

	// setup the multicast destination address
	printf("INFO      : multicast ip address %s port %d with frequency %d\n",
		conf->getMulticast().ip.c_str(), conf->getMulticast().port, conf->getMulticast().frequency);
	memset( (char *) &toAddr, 0, sizeof(toAddr) );
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr(conf->getMulticast().ip.c_str()); // multicastSend group
	toAddr.sin_port = htons(conf->getMulticast().port); // port
}

void multicastSend::send( ) {
	sendBuf.type = 0x42;
	sendBuf.cnt = packetCnt++;
	sendBuf.size = HEADER_SIZE;

	// idx is now number of bytes
	if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
		perror("ERROR  : cannot send message, message");
		exit(1);
	}
}

void multicastSend::printNetworkInterface( char *buff, int flags ) {
	printf( "INFO      : %s flags 0x%04x :", buff, flags );
	if( flags & IFF_UP ) { printf( " up"); }
	if( flags & IFF_BROADCAST ) { printf( ", broadcast"); }
	if( flags & IFF_DEBUG ) { printf( ", debug"); }
	if( flags & IFF_LOOPBACK ) { printf( ", loopback"); }
	if( flags & IFF_POINTOPOINT ) { printf( ", point-to-point"); }
	if( flags & IFF_RUNNING ) { printf( ", running"); }
	if( flags & IFF_NOARP ) { printf( ", no arp protocol"); }
	if( flags & IFF_PROMISC ) { printf( ", promiscuous mode"); }
	if( flags & IFF_NOTRAILERS ) { printf( ", avoid trailers"); }
	if( flags & IFF_ALLMULTI ) { printf( ", receive all multicast"); }
	if( flags & IFF_MASTER ) { printf( ", master load balance"); }
	if( flags & IFF_SLAVE ) { printf( ", slave load balance"); }
	if( flags & IFF_MULTICAST ) { printf( ", support multicast"); }
	if( flags & IFF_PORTSEL ) { printf( ", media type via ifmap"); }
	if( flags & IFF_AUTOMEDIA ) { printf( ", auto media"); }
	if( flags & IFF_DYNAMIC ) { printf( ", address lost when down"); }
	// if( flags & IFF_LOWER_UP ) { printf( ", signals L1"); }
	// if( flags & IFF_DORMANT ) { printf( ", signals Dormant"); }
	// if( flags & IFF_ECHO ) { printf( ", echo"); }

	printf( "\n");
}


void multicastSend::stats( ) {
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
		if( balls[ii].size >= conf->getBall( ballType ).size ) {	ballAmount++; }
	}

	uint8_t cyanAmount = 0;
	for( size_t ii = 0; ii < cyans.size(); ii++ ) {
		if( cyans[ii].size >= conf->getBall( cyanType ).size ) {	cyanAmount++; }
	}

	uint8_t magentaAmount = 0;
	for( size_t ii = 0; ii < magentas.size(); ii++ ) {
		if( magentas[ii].size >= conf->getBall( magentaType ).size ) {	magentaAmount++; }
	}

	uint8_t obstacleAmount = 0;
	for( uint ii = 0; ii < obstacles.size(); ii++ ) {
		if( obstacles[ii].size >= conf->getObstacle().size ) { obstacleAmount++; }
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
	sendBuf.pl.u16[4] = 10.0*prep->getUptime(); // uptime since application startup
	sendBuf.pl.u16[5] = 100.0*prep->getFps(); // frames per second for balls and obstacles
	sendBuf.pl.u16[6] = 100.0*loc->getFps(); // frames per second for localization
	sendBuf.pl.u32[4] = prep->getCount(); // amount of frames since application start
	sendBuf.pl.u32[5] = loc->getCount(); // amount of localizations performed since application start
	sendBuf.pl.u64[3] = localTime; // 64 bits time in micro seconds since 1 January 1970
	sendBuf.size = HEADER_SIZE + 4*8; // update with last send.pl.u64

	if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
		perror("ERROR  : cannot send message, message");
		exit(1);
	}
}

void multicastSend::locList( ) {
	vector<detPosSt> locList = detPos->getLocList();

	uint16_t payloadIndex = 0;
	size_t locListSend = locList.size();
	if( locListSend > 3 ) { locListSend = 3; } // do not send more then 3 locs

	for( size_t ii = 0; ii < locListSend; ii++ ) {
		// use fixed point with 6 bits for fraction, range x[0:976], y[0:700], rz[0:359], 2^16/1000=65
		sendBuf.pl.u16[payloadIndex++] = round(64.0*locList[ii].pos.x);
		sendBuf.pl.u16[payloadIndex++] = round(64.0*locList[ii].pos.y);
		sendBuf.pl.u16[payloadIndex++] = round(64.0*locList[ii].pos.rz);
		// use fixed point with 16 bits for fraction, range score[0:1]
		if( locList[ii].score >= 1.0 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(65535.0*locList[ii].score);
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

	// only send the localization list if one or more localization candidates have been found
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_LOCALIZATION;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

void multicastSend::goodEnoughLoc( ) {
	detPosSt goodEnoughLoc = detPos->getGoodEnoughLocExport();
	detPosSt goodEnoughLocRos = detPos->getGoodEnoughLocExportRos();

	uint16_t payloadIndex = 0;
	if( goodEnoughLoc.goodEnough ) {
		// use fixed point with 6 bits for fraction, range x[0:976], y[0:700], rz[0:359], 2^16/1000=65
		sendBuf.pl.u16[payloadIndex++] = round(64.0*goodEnoughLoc.pos.x);
		sendBuf.pl.u16[payloadIndex++] = round(64.0*goodEnoughLoc.pos.y);
		sendBuf.pl.u16[payloadIndex++] = round(64.0*goodEnoughLoc.pos.rz);

		// use fixed point with 12 bits for fraction, range x[-6.0:6.0], y[-9:9], 2^15/9=3641
		sendBuf.pl.s16[payloadIndex++] = round(2048.0*goodEnoughLocRos.pos.x);
		sendBuf.pl.s16[payloadIndex++] = round(2048.0*goodEnoughLocRos.pos.y);
		// use fixed point with 13 bits for fraction, range rz[0:6.28], 2^16/6.28=10436
		sendBuf.pl.u16[payloadIndex++] = round(8192.0*goodEnoughLocRos.pos.rz);

		// use fixed point with 16 bits for fraction, range score[0:1]
		if( goodEnoughLoc.score >= 1.0 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = round(65535.0*goodEnoughLoc.score);
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

	// only send the good enough location when it is good enough
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_GOOD_ENOUGH_LOC;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

// send the floor line points
void multicastSend::floorLinePoints( ) {
	cv::vector<cv::Point> linePointList = linePoint->getLinePoints();

	uint16_t payloadIndex = 0;
	uint16_t sendMaxPoints = linePointList.size();
	if( sendMaxPoints > 50 ) { sendMaxPoints = 50; } // TODO: make configurable
	for( size_t ii = 0; ii < sendMaxPoints; ii++) {
		sendBuf.pl.u16[payloadIndex++] = linePointList.at(ii).x;
		sendBuf.pl.u16[payloadIndex++] = linePointList.at(ii).y;
	}

	// only send the floor line points if one or more floor line points have been found
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_LINEPOINTS;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

// send the ball list, also used for the cyanList and magentaList
void multicastSend::ballList( size_t type ) {
	vector<ballSt> ballList;
	if( type == TYPE_BALLDETECTION ) {
		ballList = ballDet->getPositionsExport();
	} else if ( type == TYPE_CYANDETECTION ) {
		ballList = cyanDet->getPositionsExport();
	} else if ( type == TYPE_MAGENTADETECTION ) {
		ballList = magentaDet->getPositionsExport();
	}

	uint16_t payloadIndex = 0;
	for( size_t ii = 0; ii < ballList.size(); ii++) {
		if( ballList[ii].size >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = ballList[ii].size;
		}

		// fixed point: 65536/32 = 2000, radius range 0 to 1000
		sendBuf.pl.u16[payloadIndex++] = round((32.0*ballList[ii].radiusLin));

		// fixed point: 65536/128 = 512, angle range 0 to 360
		sendBuf.pl.u16[payloadIndex++] = round((128.0*ballList[ii].angle));
	}

	// only send the ball list if one or more balls have been found
	if( payloadIndex > 0 ) {
		sendBuf.type = type;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}

// send the obstacle list
void multicastSend::obstaclelList( ) {
	vector<obstacleSt> obstacleList = obstDet->getPositionsExport();

	uint16_t payloadIndex = 0;
	for( size_t ii = 0; ii < obstacleList.size(); ii++) {
		if( obstacleList[ii].size >= 65535 ) {
			sendBuf.pl.u16[payloadIndex++] = 65535;
		} else {
			sendBuf.pl.u16[payloadIndex++] = obstacleList[ii].size;
		}

		// fixed point: 65536/32 = 2000, radius range 0 to 1000
		sendBuf.pl.u16[payloadIndex++] = round((32.0*obstacleList[ii].radiusLin));

		// fixed point: 65536/128 = 512, angle range 0 to 360
		sendBuf.pl.u16[payloadIndex++] = round((128.0*obstacleList[ii].angle));
	}

	// only send the obstacle list if one or more obstacles have been found
	if( payloadIndex > 0 ) {
		sendBuf.type = TYPE_OBSTACLES;
		sendBuf.cnt = packetCnt++;
		sendBuf.size = HEADER_SIZE + payloadIndex * 2;

		if( sendto( fd, &sendBuf, sendBuf.size, 0, (struct sockaddr *) &toAddr, sizeof(toAddr) ) < 0 ) {
			perror("ERROR  : cannot send message, message");
			exit(1);
		}
	}
}
