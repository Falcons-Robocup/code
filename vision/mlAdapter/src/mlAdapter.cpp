// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mlAdapter.h"

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <string.h>
#include <unistd.h>

#include "falconsCommon.hpp" //getRobotNumber()
#include "FalconsRtDB2.hpp"

using namespace std;

mlAdapter::mlAdapter() {
	
	// initialize RtDB
    _rtdb = RtDB2Store::getInstance().getRtDB2( getRobotNumber() );

	// create a normal UDP socket
	if ((udpSocket = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR   cannot create UDP socket, message: %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port
	int reuse = 1;
	if (setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse)) < 0) {
		printf("ERROR   cannot configure port for multiple UDP sockets, message: %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// fill in the local address structure
	struct sockaddr_in localAddr;
	memset((void *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(46464); // TODO
	// bind to the local address / port
	if (::bind(udpSocket, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		printf("ERROR   cannot bind to UDP socket, message: %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset((void *) &mreq, 0, sizeof(mreq)); // set all to zero
	// mreq.imr_multiaddr.s_addr = inet_addr("239.255.43.21"); // multicast group on local network
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.32.74"); // multicast group on local network, TODO

	// first try IP robot configuration
	// mreq.imr_interface.s_addr = inet_addr("127.0.0.1");
	mreq.imr_interface.s_addr = inet_addr("10.0.0.65"); // TODO
	if (setsockopt(udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
		// fallback
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		if (setsockopt(udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
			printf("ERROR   cannot join the multicast group, message %s\n", strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
	}

	printf("INFO    listen on multicast group %s port %u\n", inet_ntoa(mreq.imr_multiaddr), ntohs(localAddr.sin_port));


	rxPacketCntEnabled = false;

	while( true ) {
		receive();
	}
} // constructor

void mlAdapter::receive() {
	while (true) {
		// block until data received
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		ssize_t nBytes = recvfrom(udpSocket, &rxPacket, sizeof(camPacketT), 0, (struct sockaddr *) &fromAddr,
				&fromAddrlen);
		if (nBytes < 0) {
			printf("ERROR   reading from socket, message: %s\n", strerror(errno));
			close(udpSocket);
			exit(EXIT_FAILURE);
		}

		if (rxPacket.size != nBytes) {
			printf("ERROR   received %zd bytes, but expected %u bytes\n", nBytes, rxPacket.size);
		} else {
			// printf("INFO      : received %u bytes from sender %s:%u\n", rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//        ntohs(fromAddr.sin_port));
		}

		if (rxPacket.size > 20) {
			// printf("INFO    robot %u camera %u frame %5d size %4u bytes\n", rxPacket.id >> 4, rxPacket.id & 0xf, rxPacket.pl.s32[0], rxPacket.size);
			//fflush(stdout);
			decode();
		} else {
			printf("ERROR   packet size %d to less\n", rxPacket.size);
			fflush(stdout);
		}
	} // while
} // receive

void mlAdapter::decode() {
    mlObjects.robot = rxPacket.id >> 4;
	mlObjects.camera = rxPacket.id & 0x0f;
	mlObjects.milliSeconds = rxPacket.pl.u64[0];
	mlObjects.frame = rxPacket.pl.s32[2];

	if( rxPacketCntEnabled && ( rxPacketCntExpected != rxPacket.cnt ) ) {
		printf("ERROR   packet counter is %u, but expected %u\n", rxPacket.cnt, rxPacketCntExpected);
	}
	rxPacketCntEnabled = true;
	rxPacketCntExpected = rxPacket.cnt + 1;


	mlObjects.objects.clear();

	visionObject obj;

	bool keepGoing = true;
	int ii = 3; // first 3 * 4 bytes used for msec and frameId
	while (keepGoing) {
		obj.azimuth = rxPacket.pl.f32[ii];
		ii++;
		obj.classId = rxPacket.pl.s32[ii];
		ii++;
		obj.confidence = rxPacket.pl.f32[ii];
		ii++;
		obj.elevation = rxPacket.pl.f32[ii];
		ii++;
		obj.height = rxPacket.pl.f32[ii];
		ii++;
		obj.radius = rxPacket.pl.f32[ii];
		ii++;
		obj.width = rxPacket.pl.f32[ii];
		ii++;
		obj.xCenter = rxPacket.pl.f32[ii];
		ii++;
		obj.yCenter = rxPacket.pl.f32[ii];
		ii++; // 36 bytes per object

		mlObjects.objects.push_back(obj);

		int nextEnd = 4 + ii * 4 + 36; // header + done + newOne

		if (nextEnd > rxPacket.size) {
			keepGoing = false;
			if (nextEnd != (rxPacket.size + 36)) {
				printf("ERROR   nextEnd %d, rxPacketSize %d, delta %d\n", nextEnd, rxPacket.size,
						nextEnd - rxPacket.size);
			}
		}
	} // while

	print();

    _rtdb->put(VISION_OBJECTS, &mlObjects);
} // decode

void mlAdapter::print() {
	if ( (mlObjects.frame % 10 ) == 0) {
		time_t remoteEpochTimeSec = mlObjects.milliSeconds / 1000; // extract the seconds and convert to time_t which is required for localtime
		struct tm * remoteTimeSec;
		remoteTimeSec = localtime(&remoteEpochTimeSec);
		char remoteTimeString[16];
		strftime(remoteTimeString, 16, "%H:%M:%S", remoteTimeSec);

		uint64_t fraction = mlObjects.milliSeconds % 1000;

		printf("INFO    %8s.%03zu robot %d cam %d frame %6d objects %2zu\n", remoteTimeString, fraction, mlObjects.robot, mlObjects.camera,
				mlObjects.frame, mlObjects.objects.size());

		// TODO: print object list

		fflush(stdout);
	}
}

int main() {
	mlAdapter adapt;
	return 0;
}
