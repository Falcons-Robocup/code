// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mlAdapter.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <string.h>
#include <unistd.h> // getopt

#include "falconsCommon.hpp" //getRobotNumber()
#include "FalconsRTDB.hpp"
#include "rtdbKeys.hpp"

using namespace std;

mlAdapter::mlAdapter(bool quiet, bool testMode) {
	this->quiet = quiet;
	this->testMode = testMode;

	// initialize RtDB
	_rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(getRobotNumber());

	// int status = system("route | grep 224.16.16.0 && sudo route del -net 224.16.16.0 netmask 255.255.255.0");
	// printf("INFO   status del %d\n", status);
	string interface = getInterface();
	string addRoute = "sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev " + interface;
	printf("INFO    %s\n", addRoute.c_str());
	string command = "route | grep 224.16.16.0 || " + addRoute;
	printf("INFO    ");
	fflush(stdout);
	if( system(command.c_str()) != 0 ) {
		printf("ERROR   cannot set route\n");
		printf("        %s\n", addRoute.c_str());
		exit(EXIT_FAILURE);
	}

	// create a normal UDP socket
	if( (udpSocket = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
		printf("ERROR   cannot create UDP socket, message: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port
	int reuse = 1;
	if( setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse)) < 0 ) {
		printf("ERROR   cannot configure port for multiple UDP sockets, message: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}

	// fill in the local address structure
	struct sockaddr_in localAddr;
	memset((void*)&localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(46464); // TODO
	// bind to the local address / port
	if( ::bind(udpSocket, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0 ) {
		printf("ERROR   cannot bind to UDP socket, message: %s\n", strerror(errno));
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset((void*)&mreq, 0, sizeof(mreq)); // set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network

	// first try IP robot configuration
	// mreq.imr_interface.s_addr = inet_addr("127.0.0.1");
	mreq.imr_interface.s_addr = inet_addr("10.0.0.65"); // TODO
	if( setsockopt(udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0 ) {
		// fallback
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		if( setsockopt(udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0 ) {
			printf("ERROR   cannot join the multicast group, message %s\n", strerror(errno));
			exit(EXIT_FAILURE);
		}
	}

	printf("INFO    listen on multicast group %s port %u\n", inet_ntoa(mreq.imr_multiaddr), ntohs(localAddr.sin_port));

	rxPacketCntEnabled = false;
	frameCounter = 0;

	while( true ) {
		if( testMode ) {
			testGenerator();
		} else {
			receive();
		}
	}
} // constructor

string mlAdapter::getInterface() {
	char buff[64] = { 0x0 };
	FILE *cmd = popen("ls /sys/class/net | grep en", "r");
	while( fgets(buff, sizeof(buff), cmd) != NULL ) {
	}
	pclose(cmd);

	string interface = buff;
	interface.resize(strlen(buff) - 1);
	// printf("INFO    interface _%s_\n", interface.c_str());
	return interface;
}

void mlAdapter::receive() {
	while( true ) {
		// block until data received
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		ssize_t nBytes = recvfrom(udpSocket, &rxPacket, sizeof(camPacketT), 0, (struct sockaddr*)&fromAddr,
				&fromAddrlen);
		if( nBytes < 0 ) {
			printf("ERROR   reading from socket, message: %s\n", strerror(errno));
			close(udpSocket);
			exit(EXIT_FAILURE);
		}

		if( rxPacket.size != nBytes ) {
			printf("ERROR   received %zd bytes, but expected %u bytes\n", nBytes, rxPacket.size);
		} else {
			// printf("INFO      : received %u bytes from sender %s:%u\n", rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//        ntohs(fromAddr.sin_port));
		}

		if( rxPacket.size >= 16 ) { // header + milliSeconds + frameCounter
			// printf("INFO    robot %u frame %5d size %4u bytes\n", rxPacket.id, rxPacket.pl.s32[0], rxPacket.size);
			// fflush(stdout);
			decode();
		} else {
			printf("ERROR   packet size %d to less\n", rxPacket.size);
		}
	} // while
} // receive

void mlAdapter::decode() {
	vFrame.robot = rxPacket.id;
	struct timeval tv = { 0 };
	tv.tv_sec = rxPacket.pl.u64[0] / 1000;
	tv.tv_usec = 1000 * (rxPacket.pl.u64[0] % 1000);
	vFrame.timestamp.fromTimeval(tv);
	vFrame.frame = rxPacket.pl.s32[2];

	if( rxPacketCntEnabled && (rxPacketCntExpected != rxPacket.cnt) ) {
		printf("ERROR   packet counter is %u, but expected %u\n", rxPacket.cnt, rxPacketCntExpected);
	}
	rxPacketCntEnabled = true;
	rxPacketCntExpected = rxPacket.cnt + 1;

	vFrame.objects.clear();

	visionObject obj;

	bool keepGoing = false;
	if( rxPacket.size >= (4 + 12 + 28) ) {
		keepGoing = true;
	}
	int ii = 3; // first 3 * 4 bytes used for milliseconds and frameId

	while( keepGoing ) {
		// location is relative to the field (use x, y and azimuth) (coordinate 0,0 is center of field, TODO: define azimuth reference)
		// obstacle is relative to the location (use azimuth, elevation and radius)
		obj.azimuth = rxPacket.pl.f32[ii]; // 0 to 2 Pi, TODO: define to where 0 points
		ii++;
		int classId = rxPacket.pl.s32[ii]; // see below
		ii++;
		obj.confidence = rxPacket.pl.f32[ii]; // 0 to 1
		ii++;
		obj.elevation = rxPacket.pl.f32[ii]; // -Pi to Pi, 0 is horizontal
		ii++;
		obj.height = 0.0;
		obj.radius = rxPacket.pl.f32[ii]; // in meters
		ii++;
		obj.width = 0.0;
		obj.xCenter = rxPacket.pl.f32[ii]; // xLocation in meters
		ii++;
		obj.yCenter = rxPacket.pl.f32[ii]; // yLocation in meters
		ii++; // 28 bytes per object

		// convert class number to class name
		if( classId == -1 ) {
			obj.className = "location";
		} else if( classId == 0 ) {
			obj.className = "ball";
		} else if( classId == 1 ) {
			obj.className = "obstacle";
		} else if( classId == 2 ) {
			obj.className = "human";
		} else if( classId == 3 ) {
			obj.className = "goalPost";
		} else if( classId == 4 ) {
			obj.className = "border";
		} else {
			obj.className = "unknown";
		}

		vFrame.objects.push_back(obj);

		int nextEnd = 4 + ii * 4 + 28; // header + done + newOne

		if( nextEnd > rxPacket.size ) {
			keepGoing = false;
			if( nextEnd != (rxPacket.size + 28) ) {
				printf("ERROR   nextEnd %d, rxPacketSize %d, delta %d\n", nextEnd, rxPacket.size,
						nextEnd - rxPacket.size);
			}
		}
	} // while

	_rtdb->put(VISION_FRAME, &vFrame);

	// heartbeat put will trigger worldmodel to update
	static int heartbeat_counter = 0;
	heartbeat_counter++;
	_rtdb->put(HEARTBEAT_WORLDMODEL, &heartbeat_counter);

	if( !quiet ) {
		printFrame();
	}
} // decode

void mlAdapter::testGenerator() {

	struct timeval tp;
	gettimeofday(&tp, NULL);

	vFrame.robot = getRobotNumber();
	vFrame.timestamp.fromTimeval(tp);
	vFrame.frame = frameCounter;
	frameCounter++;

	vFrame.objects.clear();

	visionObject obj;
	obj.azimuth = -0.79;
	obj.className = "ball";
	obj.confidence = 0.90;
	obj.elevation = -0.64; // 1m distance
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 1.0;
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = 0.79;
	obj.className = "ball";
	obj.confidence = 0.85;
	obj.elevation = -0.36; // 2m distance
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 2.0;
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = -0.79;
	obj.className = "obstacle";
	obj.confidence = 0.75;
	obj.elevation = -0.36; // 2m distance
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 2.0;
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = 0.79;
	obj.className = "obstacle";
	obj.confidence = 0.65;
	obj.elevation = -0.19; // 4m distance
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 4.0;
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = 0.0;
	obj.className = "human";
	obj.confidence = 0.70;
	obj.elevation = -0.124; // 6m distance
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 6.0;
	obj.width = 0.0;  // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = 0.53;
	obj.className = "goalPost";
	obj.confidence = 0.80;
	obj.elevation = -0.25;
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 2.79;
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = -0.91;
	obj.className = "border";
	obj.confidence = 0.91;
	obj.elevation = -0.75;
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 1.24;
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.0; // not currently used by worldmodel
	obj.yCenter = 0.0; // not currently used by worldmodel
	vFrame.objects.push_back(obj);

	obj.azimuth = 0.10;
	obj.className = "location";
	obj.confidence = 0.93;
	obj.elevation = 0.0; // not currently used by worldmodel
	obj.height = 0.0; // not currently used by worldmodel
	obj.radius = 0.0; // not currently used by worldmodel
	obj.width = 0.0; // not currently used by worldmodel
	obj.xCenter = 0.30;
	obj.yCenter = 0.18;
	vFrame.objects.push_back(obj);

	_rtdb->put(VISION_FRAME, &vFrame);

	// heartbeat put will trigger worldmodel to update
	static int heartbeat_counter = 0;
	heartbeat_counter++;
	_rtdb->put(HEARTBEAT_WORLDMODEL, &heartbeat_counter);

	if( !quiet ) {
		printFrame();
	}

	usleep(250000); // 4 fps
}

void mlAdapter::printFrame() {
	if( (vFrame.frame % 10) == 0 ) {

		printf("INFO    %s robot %d frame %6d objects %2zu\n", vFrame.timestamp.toStr().c_str(), vFrame.robot,
				vFrame.frame, vFrame.objects.size());

		for( size_t ii = 0; ii < vFrame.objects.size(); ii++ ) {
			visionObject obj = vFrame.objects[ii];
			if( obj.className == "location" ) {
				printf("           %-10s  conf %3.0f%%  x %5.2f  y %5.2f  azimuth %5.2f\n", obj.className.c_str(),
						100.0 * obj.confidence, obj.xCenter, obj.yCenter, obj.azimuth);
			} else if( obj.radius > 99.99 ) {
				printf(
						"           %-10s  conf %3.0f%%                    azimuth %5.2f  elevation %5.2f  radius large\n",
						obj.className.c_str(), 100.0 * obj.confidence, obj.azimuth, obj.elevation);
			} else {
				printf(
						"           %-10s  conf %3.0f%%                    azimuth %5.2f  elevation %5.2f  radius %3.2f\n",
						obj.className.c_str(), 100.0 * obj.confidence, obj.azimuth, obj.elevation, obj.radius);
			}
		}
	}
}

void help() {
	printf("INFO    arguments\n");
	printf("          -q quiet\n");
	printf("          -t generate test objects\n");
}

int main(int argc, char **argv) {
	int opt = 0;
	bool test = false;
	bool quiet = false;
	while( (opt = getopt(argc, argv, "qth")) != -1 ) {
		switch( opt ) {
		case 'q':
			quiet = true;
			break;
		case 't':
			test = true;
			printf("INFO    test mode\n");
			break;
		case 'h':
			help();
			exit(EXIT_SUCCESS);
			break;
		default:
			printf("ERROR   invalid arguments\n");
			help();
			exit(EXIT_FAILURE);
			break;
		} // case
	} // while

	mlAdapter adapt(quiet, test);
	return 0;
}
