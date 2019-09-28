// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "multicastReceive.hpp"

// #define READFROMFILE

using namespace std;

multicastReceive::multicastReceive() {

	packetIndex = 0;
	packetIndexPrev = 1; // should be different from packetIndex to be able to use also the first packet
	packetIndexPause = false;

	struct timeval tv;
	time_t nowtime;
	struct tm *nowtm;

	gettimeofday(&tv, NULL);
	nowtime = tv.tv_sec;
	nowtm = localtime(&nowtime);
	// use current date and time as record file name
	char writeTxtName[64], writeBinName[64];
	strftime(writeTxtName, sizeof writeTxtName, "%Y-%m-%d_%H-%M-%S.log", nowtm);
	strftime(writeBinName, sizeof writeBinName, "%Y-%m-%d_%H-%M-%S.bin", nowtm);
	printf("INFO      : write received data to text file %s and binary file %s\n", writeTxtName, writeBinName);

	if ((writeTxt = fopen(writeTxtName, "wb")) == NULL) { // also use binary for ASCII
		perror("ERROR     : cannot open record text file for writing, message");
		exit(EXIT_FAILURE);
	}
	if ((writeBin = fopen(writeBinName, "wb")) == NULL) {
		perror("ERROR     : cannot open record binary file for writing, message");
		exit(EXIT_FAILURE);
	}

#ifdef READFROMFILE

	string logFile;
	// logFile = "matches/torres_vedras_cambada_2018-04-25_21-01-50.bin";
	// logFile = "matches/torres_vedras_cambada_2018-04-25_21-31-21.bin";
	// logFile = "matches/2018-04-28_16-19-04_robot_6_test_cam_2_issue.bin";
	// logFile = "matches/2018-04-28_19-16-48_robot_6_ghost_ball.bin";
	// logFile = "matches/tech_united_2_2018-06-20_01-14-32.bin";
	// logFile = "matches/water_1_2018-06-20_17-11-59.bin";
	// logFile = "2019-01-13_11-28-35.bin";
	// logFile = "matches/2019-04-26_14-35-48_techunited.bin";
	// logFile = "matches/2019-04-28_11-46-15_techunited.bin";
	// logFile = "matches/2019-07-04_03-31-42_nubot.bin";
	// logFile = "matches/2019-07-06_00-52-51_nubot.bin";
	// logFile = "matches/2019-07-06_05-48-32_iris.bin";
	logFile = "matches/2019-07-06_09-26-44_hibikino.bin";

	if ((readBin = fopen(logFile.c_str(), "r")) == NULL) {
		printf("ERROR     : READ FROM FILE MODE, cannot open file file %s for reading, message, %s\n", logFile.c_str(),
				strerror(errno));
		exit(EXIT_FAILURE);
	}

	size_t len = 0;
	char * line = NULL;
	ssize_t read = 0;

	while ((read = getline(&line, &len, readBin)) != -1) {
		packetSt packet;
		// WARNING: one value is 2 bytes
		for (int ii = 0; ii < (read - 1) / 2; ii++) { // TODO: double check range
			char substr[2]; // each hex value is 2 chars
			strncpy(substr, line + (ii * 2), 2); // copy 2 chars
			uint8_t value = (uint8_t) strtol(substr, NULL, 16); // convert 2 chars to long
			if (ii == 0) {
				packet.index = value;
			} else {
				packet.data.push_back(value);
			}
		}
		packets.push_back(packet);
	}
	free(line);
	fclose(readBin);
	printf("INFO      : have read %zu packets from file\n", packets.size());

#ifdef NONO
	// check the stored packet
	size_t maxPackets = packets.size();
	if( maxPackets > 5 ) {maxPackets = 5;}
	for( size_t ii = 0; ii < maxPackets; ii ++ ) {
		printf("%2zx ", packets[ii].index);
		for( size_t jj = 0; jj < packets[ii].data.size(); jj++ ) {
			printf("%02x", packets[ii].data[jj]);
		}
		printf("\n");
	}
#endif

#endif

	struct passwd *pw = getpwuid(getuid());
	string configFile("");
	configFile.append(pw->pw_dir);
	configFile.append("/falcons/code/packages/multiCam/multiCam.yaml");
	printf("INFO      : multicastSend uses configuration file: %s\n", configFile.c_str());
	cv::FileStorage fs(configFile, cv::FileStorage::READ);

	cv::FileNode possession = fs["possession"];
	possessionThreshold = (int) (possession["threshold"]);

#ifndef READFROMFILE

	// the multicast address and port number are in the global section of yaml file
	cv::FileNode global = fs["global"];
	string multicastIp = (string) (global["multicastIp"]);
	int multicastPort = (int) (global["multicastPort"]);
	printf("INFO      : multicastSend ip address %s port %d\n", multicastIp.c_str(), multicastPort);

	// create a normal UDP socket
	if ((fd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cannot create UDP socket %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port
	int reuse = 1;
	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse)) < 0) {
		printf("ERROR     : cannot configure port for multiple UTP sockets, %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// fill in the local address struct
	struct sockaddr_in localAddr;
	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;// Internet
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(multicastPort);// port
	// bind to the local address / port
	if (bind(fd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		printf("ERROR     : cannot bind to UDP socket, %s", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset(&mreq, 0, sizeof(mreq));// set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr(multicastIp.c_str());
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
		printf("ERROR     : cannot join the multicast group, %s\n", strerror(errno));
		printf("INFO      : goto Edit Connections\n");
		printf("INFO      :   select Wi-Fi MSL_FIELD_A_a\n");
		printf("INFO      :     select IPv4 Settings\n");
		printf("INFO      :       verify address is 172.16.74.x\n");
		printf("INFO      :       verify netmask 255.255.255.0 or 24\n");
		printf("INFO      :       leave gateway open\n");
		printf("INFO      :     select Routes\n");
		printf("INFO      :       add\n");
		printf("INFO      :         address 224.16.32.0\n");
		printf("INFO      :         netmask 255.255.255.0\n");
		printf("INFO      :         leave gateway open\n");
		printf("INFO      :         metric 0\n");
		printf("INFO      :       ok\n");
		printf("INFO      :     save\n");
		printf("INFO      : reconnect to MSL_FIELD_A_a\n");
		printf("INFO      : verify with route -n if route to 224.26.32.0 exists\n");
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	fromAddrLen = sizeof(fromAddr);

	printf("INFO      : camera receive multicast group address %s port %u\n", inet_ntoa(mreq.imr_multiaddr),
			ntohs(localAddr.sin_port));

#endif

	// create the packetCnt for the 6 robots
	uint8_t tmpPacketCnt = 0;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		packetCnt.push_back(tmpPacketCnt);
	}

	// create the packet Error Count for the 6 robots
	size_t tmpPacketErrorCnt = 0;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		packetErrorCnt.push_back(tmpPacketErrorCnt);
	}

	// create the statistics for the 6 robots
	raspiStatsSt tmpRaspi = { 0, 60, 32, 0, 60, 0, 0, 0, 0, 10, 0 };
	statsSt tmpStat = { 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0, "", tmpRaspi, tmpRaspi, tmpRaspi, tmpRaspi };
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		tmpStat.robotIndex = ii; // starting at 0 instead of 1
		stats.push_back(tmpStat);
	}

	// create the linePointList for the 6 robots
	ballObstListSt tmpLinePointList;
	tmpLinePointList.packetCnt = 0;
	tmpLinePointList.state = invalid;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		linePointList.push_back(tmpLinePointList);
	}

	// create the goodEnoughLoc for the 6 robots
	goodEnoughLocSt tmpGoodEnoughLoc;
	tmpGoodEnoughLoc.packetCnt = 0;
	tmpGoodEnoughLoc.state = invalid;
	tmpGoodEnoughLoc.goodEnough = false; // default not good enough
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		goodEnoughLoc.push_back(tmpGoodEnoughLoc);
	}

	positionStDbl tmpPos;
	tmpPos.x = 0.0;
	tmpPos.y = 0.0;
	tmpPos.rz = 0.0;
	vector<positionStDbl> tmpPosList;
	// next line set's the amount of measurements used for the average
	for (size_t ii = 0; ii < 20; ii++) {
		tmpPosList.push_back(tmpPos);
	}
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		posRosHistory.push_back(tmpPosList);
		posRosAverage.push_back(tmpPos);
	}

	// create the locList for the 6 robots
	locListSt tmpLocList;
	tmpLocList.packetCnt = 0;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		locList.push_back(tmpLocList);
	}

	// create the ballList and ballFarList for the 6 robots
	ballObstListSt tmpBallList;
	tmpBallList.packetCnt = 0;
	tmpBallList.state = invalid;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		ballList.push_back(tmpBallList);
		ballFarList.push_back(tmpBallList);
	}

	// create the cyanList for the 6 robots
	ballObstListSt tmpCyanList;
	tmpCyanList.packetCnt = 0;
	tmpCyanList.state = invalid;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		cyanList.push_back(tmpCyanList);
	}

	// create the magentaList for the 6 robots
	ballObstListSt tmpMagentaList;
	tmpMagentaList.packetCnt = 0;
	tmpMagentaList.state = invalid;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		magentaList.push_back(tmpMagentaList);
	}

	// create the obstacleList for the 6 robots
	ballObstListSt tmpObstacleList;
	tmpObstacleList.packetCnt = 0;
	tmpObstacleList.state = invalid;
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		obstacleList.push_back(tmpObstacleList);
	}

	statsExportMutex.unlock();
	linePointListExportMutex.unlock();
	goodEnoughLocExportMutex.unlock();
	locListExportMutex.unlock();
	ballListExportMutex.unlock();
	ballFarListExportMutex.unlock();
	cyanListExportMutex.unlock();
	magentaListExportMutex.unlock();
	obstacleListExportMutex.unlock();
}

multicastReceive::~multicastReceive() {
	fclose(writeTxt);
	fclose(writeBin);
}

void multicastReceive::decodeStats(size_t index) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t localTime = tv.tv_sec * 1000000 + tv.tv_usec;

	uint8_t statsPacketSize = HEADER_SIZE + 32 + 4 * 20;
	if (recvBuf.size != statsPacketSize) {
		printf(" %10s : ERROR     : stats packet should be %d bytes, but received %d bytes\n", ipAddress,
				statsPacketSize, recvBuf.size);
	} else {
		statsExportMutex.lock();
		stats[index].packetCnt = recvBuf.cnt;
		stats[index].ballAmount = recvBuf.pl.u8[0 / 1];
		stats[index].cyanAmount = recvBuf.pl.u8[1 / 1];
		stats[index].magentaAmount = recvBuf.pl.u8[2 / 1];
		stats[index].obstacleAmount = recvBuf.pl.u8[3 / 1];
		stats[index].possessionPixels = recvBuf.pl.u16[4 / 2]; // amount of ball pixels seen just before the shooter
		stats[index].linePoints = recvBuf.pl.u16[6 / 2]; // total amount of line points found
		stats[index].uptime = recvBuf.pl.u16[8 / 2] / 10.0; // uptime since application startup
		stats[index].prepFps = recvBuf.pl.u16[10 / 2] / 100.0; // frames per second for balls and obstacles
		stats[index].locFps = recvBuf.pl.u16[12 / 2] / 100.0; // frames per second for localization
		stats[index].prepFrames = recvBuf.pl.u32[16 / 4]; // amount of frames since application start
		stats[index].locFrames = recvBuf.pl.u32[20 / 4]; // amount of localizations performed since application start
		stats[index].remoteTime = recvBuf.pl.u64[24 / 8]; // 64 bits time in micro seconds since 1 January 1970
		stats[index].localTime = localTime; // used to calculate the transport latency (if remote has the same time)

		for (size_t camIndex = 0; camIndex < 4; camIndex++) {
			raspiStatsSt raspi;
			raspi.cmosTemp = recvBuf.pl.s8[(32 + camIndex * 16) / 1];
			raspi.cpuLoad = recvBuf.pl.u8[(33 + camIndex * 16) / 1];
			raspi.cpuTemp = recvBuf.pl.u8[(34 + camIndex * 16) / 1];
			raspi.gpuTemp = recvBuf.pl.u8[(35 + camIndex * 16) / 1];
			raspi.cpuStatus = recvBuf.pl.u8[(36 + camIndex * 16) / 1];
			raspi.camValAverage = recvBuf.pl.u8[(37 + camIndex * 16) / 1];
			raspi.rebootReceivedAge = recvBuf.pl.u8[(38 + camIndex * 16) / 1];
			// free = recvBuf.pl.u8[(39 + camIndex * 16) / 1];
			raspi.sysApplUptime = recvBuf.pl.u16[(40 + camIndex * 16) / 2];
			raspi.cpuUptime = recvBuf.pl.u16[(42 + camIndex * 16) / 2];
			raspi.anaApplUptime = recvBuf.pl.u16[(44 + camIndex * 16) / 2];
			// free = recvBuf.pl.u32[(46 + camIndex * 16) / 4];
			raspi.anaFrameCounter = recvBuf.pl.u32[(48 + camIndex * 16) / 4];
			stats[index].raspi[camIndex] = raspi;
		}

		// convert the remote epoch microsecond time to a readable string
		time_t remoteEpochTimeSec = stats[index].remoteTime / 1000000; // extract the seconds and convert to time_t which is required for localtime()
		struct tm * remoteTimeSec;
		remoteTimeSec = localtime(&remoteEpochTimeSec);
		char remoteTimeString[10];
		strftime(remoteTimeString, 10, "%H:%M:%S", remoteTimeSec);

		uint32_t remoteDotSeconds = (uint32_t) ((stats[index].remoteTime / 100000) % 10); // extract the 0.1 seconds
		sprintf(stats[index].remoteTimeString, "%s.%01u", remoteTimeString, remoteDotSeconds);
		statsExportMutex.unlock();

		int64_t deltaTime = stats[index].localTime - stats[index].remoteTime;

		// printf("local time 0x%016zx remote time 0x%016zx, delta 0x%016zx %ld\n", localTime, stats[index].remoteTime, deltaTime, deltaTime );
#ifdef NONO
		printf(" %10s stat %3d %5d : %3d %3d %3d %3d %5d %4d : %6.1f %3.1f %3.1f : %6d %6d : %5.1f ms", ipAddress,
				stats[index].packetCnt, recvBuf.size, stats[index].ballAmount, stats[index].cyanAmount,
				stats[index].magentaAmount, stats[index].obstacleAmount, stats[index].possessionPixels,
				stats[index].linePoints, stats[index].uptime, stats[index].prepFps, stats[index].locFps,
				stats[index].prepFrames, stats[index].locFrames, deltaTime / 1000.0);

		for (size_t camIndex = 0; camIndex < 4; camIndex++) {
			raspiStatsSt raspi = stats[index].raspi[camIndex];
			hhMmSsSt sysApplUp = secondsToHhMmSs(raspi.sysApplUptime);
			hhMmSsSt anaApplUp = secondsToHhMmSs(raspi.anaApplUptime);
			hhMmSsSt cpuUp = secondsToHhMmSs(raspi.cpuUptime);
			printf(" : %2u %2u %2u %3.2f 0x%02x %7u %2u %02u:%02u:%02u %02u:%02u:%02u %02u:%02u:%02u",
					raspi.cpuTemp, raspi.gpuTemp, raspi.cmosTemp, raspi.cpuLoad / 32.0, raspi.cpuStatus, raspi.anaFrameCounter, raspi.rebootReceivedAge,
					cpuUp.hours, cpuUp.minutes, cpuUp.seconds, sysApplUp.hours, sysApplUp.minutes, sysApplUp.seconds, anaApplUp.hours,
					anaApplUp.minutes, anaApplUp.seconds);

		}
		printf("\n");

#endif

		fprintf(writeTxt, "%zu %s %10s stat %3d %5d : %3d %3d %3d %3d %5d %4d : %6.1f %3.1f %3.1f : %6d %6d : %5.1f ms",
				stats[index].remoteTime, stats[index].remoteTimeString, ipAddress, stats[index].packetCnt, recvBuf.size,
				stats[index].ballAmount, stats[index].cyanAmount, stats[index].magentaAmount,
				stats[index].obstacleAmount, stats[index].possessionPixels, stats[index].linePoints,
				stats[index].uptime, stats[index].prepFps, stats[index].locFps, stats[index].prepFrames,
				stats[index].locFrames, deltaTime / 1000.0);

		for (size_t camIndex = 0; camIndex < 4; camIndex++) {
			raspiStatsSt raspi = stats[index].raspi[camIndex];
			hhMmSsSt sysApplUp = secondsToHhMmSs(raspi.sysApplUptime);
			hhMmSsSt anaApplUp = secondsToHhMmSs(raspi.anaApplUptime);
			hhMmSsSt cpuUp = secondsToHhMmSs(raspi.cpuUptime);
			fprintf(writeTxt, " : %2u %2u %2u %3.2f 0x%02x %7u %2u %02u:%02u:%02u %02u:%02u:%02u %02u:%02u:%02u",
					raspi.cpuTemp, raspi.gpuTemp, raspi.cmosTemp, raspi.cpuLoad / 32.0, raspi.cpuStatus,
					raspi.anaFrameCounter, raspi.rebootReceivedAge, cpuUp.hours, cpuUp.minutes, cpuUp.seconds,
					sysApplUp.hours, sysApplUp.minutes, sysApplUp.seconds, anaApplUp.hours, anaApplUp.minutes,
					anaApplUp.seconds);

		}
		fprintf(writeTxt, "\n");

	}

// the statistics is always transmitted by the sender (and in most cases received)
// use the packets counter to determine if the balls or objects has not been updated in a while
	int timeInvalid = 15;
	int timeOld = 6;

	linePointListExportMutex.lock();
	if (linePointList[index].state != invalid) {
		int deltaPacketCnt = (256 + linePointList[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			linePointList[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			linePointList[index].state = old;
		}
	}
	linePointListExportMutex.unlock();

	goodEnoughLocExportMutex.lock();
	if (goodEnoughLoc[index].state != invalid) {
		int deltaPacketCnt = (256 + goodEnoughLoc[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			goodEnoughLoc[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			goodEnoughLoc[index].state = old;
		}
	}
	goodEnoughLocExportMutex.unlock();

	ballListExportMutex.lock();
	if (ballList[index].state != invalid) {
		int deltaPacketCnt = (256 + ballList[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			ballList[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			ballList[index].state = old;
		}
	}
	ballListExportMutex.unlock();

	ballFarListExportMutex.lock();
	if (ballFarList[index].state != invalid) {
		int deltaPacketCnt = (256 + ballFarList[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			ballFarList[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			ballFarList[index].state = old;
		}
	}
	ballFarListExportMutex.unlock();

	cyanListExportMutex.lock();
	if (cyanList[index].state != invalid) {
		int deltaPacketCnt = (256 + cyanList[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			cyanList[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			cyanList[index].state = old;
		}
	}
	cyanListExportMutex.unlock();

	magentaListExportMutex.lock();
	if (magentaList[index].state != invalid) {
		int deltaPacketCnt = (256 + magentaList[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			magentaList[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			magentaList[index].state = old;
		}
	}
	magentaListExportMutex.unlock();

	obstacleListExportMutex.lock();
	if (obstacleList[index].state != invalid) {
		int deltaPacketCnt = (256 + obstacleList[index].packetCnt - stats[index].packetCnt) % 256; // range 0 to 255
		if ((deltaPacketCnt > timeInvalid) && (deltaPacketCnt < (256 - timeInvalid))) {
			obstacleList[index].state = invalid;
		} else if ((deltaPacketCnt > timeOld) && (deltaPacketCnt < (256 - timeOld))) {
			obstacleList[index].state = old;
		}
	}
	obstacleListExportMutex.unlock();
}

void multicastReceive::decodeLocalization(size_t index) {
// check if the packet size makes sense
	uint8_t oneLocalizationSize = 16; // each localization requires 8 * uint16_t = 16 bytes of the packet
	if (((recvBuf.size - HEADER_SIZE) % oneLocalizationSize) != 0) {
		printf(" %10s : ERROR     : localization packet should be %d + n * %d bytes, but received %d bytes\n",
				ipAddress, HEADER_SIZE, oneLocalizationSize, recvBuf.size);
	} else {
		uint16_t payloadIndex = 0;

		locListExportMutex.lock();
		locList[index].packetCnt = recvBuf.cnt;
		locList[index].list.clear();

		ssize_t amountOfLocalizations = (recvBuf.size - HEADER_SIZE) / oneLocalizationSize;
		for (ssize_t ii = 0; ii < amountOfLocalizations; ii++) {
			detPosSt loc;
			// maximal x field size in pixels : 22 + xx = 24 meter / 2 cm = 1200
			// maximal y field size in pixels : 14 + xx = 16 meter / 2 cm = 800
			// use fixed point with 5 bits for fraction, range x[0:1199], y[0:799], 2^16/1200=54
			loc.pos.x = recvBuf.pl.u16[payloadIndex++] / 32.0;
			loc.pos.y = recvBuf.pl.u16[payloadIndex++] / 32.0;
			// use fixed point with 6 bits for fraction, rz[0:359]
			loc.pos.rz = recvBuf.pl.u16[payloadIndex++] / 64.0;
			// use fixed point with 16 bits for fraction, range score[0:1]
			loc.score = recvBuf.pl.u16[payloadIndex++] / 65535.0;
			loc.lastActive = recvBuf.pl.u16[payloadIndex++];
			loc.age = recvBuf.pl.u16[payloadIndex++];
			loc.amountOnFloor = recvBuf.pl.u16[payloadIndex++];
			loc.numberOfTries = recvBuf.pl.u16[payloadIndex++];
			locList[index].list.push_back(loc);
		}
		locListExportMutex.unlock();

#ifdef NONO
		printf(" %10s loca %3d %5d", ipAddress, locList[index].packetCnt, recvBuf.size);
		for( size_t ii = 0; ii < locList[index].list.size(); ii++) {
			printf(" : %4.0f %4.0f %3.0f  %3.3f  %3d %3d %3d %3d",
					locList[index].list[ii].pos.x, locList[index].list[ii].pos.y, locList[index].list[ii].pos.rz,
					locList[index].list[ii].score,
					locList[index].list[ii].lastActive, locList[index].list[ii].age,
					locList[index].list[ii].amountOnFloor, locList[index].list[ii].numberOfTries );
		}
		printf("\n");
#endif

		fprintf(writeTxt, "%zu %s %10s loca %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString, ipAddress,
				locList[index].packetCnt, recvBuf.size);
		for (size_t ii = 0; ii < locList[index].list.size(); ii++) {
			fprintf(writeTxt, " : %4.0f %4.0f %3.0f  %3.3f  %3d %3d %3d %3d", locList[index].list[ii].pos.x,
					locList[index].list[ii].pos.y, locList[index].list[ii].pos.rz, locList[index].list[ii].score,
					locList[index].list[ii].lastActive, locList[index].list[ii].age,
					locList[index].list[ii].amountOnFloor, locList[index].list[ii].numberOfTries);
		}
		fprintf(writeTxt, "\n");
	}
}

void multicastReceive::decodeGoodEnoughLoc(size_t index) {
// check if the packet size makes sense
	uint8_t goodEnoughSize = 3 * 2 + 2 * 2 + 2 + 5 * 2; // see below for the size
	if (recvBuf.size != HEADER_SIZE + goodEnoughSize) {
		printf(" %10s : ERROR     : goodEnough packet should be %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE + goodEnoughSize, recvBuf.size);
	} else {
		uint16_t payloadIndex = 0;

		goodEnoughLocExportMutex.lock();
		goodEnoughLoc[index].packetCnt = recvBuf.cnt;
		goodEnoughLoc[index].state = updated;

		// maximal x field size in pixels : 22 + xx = 24 meter / 2 cm = 1200
		// maximal y field size in pixels : 14 + xx = 16 meter / 2 cm = 800
		// use fixed point with 5 bits for fraction, range x[0:1199], y[0:799], 2^16/1200=54
		goodEnoughLoc[index].pos.x = recvBuf.pl.u16[payloadIndex++] / 32.0;
		goodEnoughLoc[index].pos.y = recvBuf.pl.u16[payloadIndex++] / 32.0;
		// use fixed point with 6 bits for fraction, range rz[0:359]
		goodEnoughLoc[index].pos.rz = recvBuf.pl.u16[payloadIndex++] / 64.0;

		// use fixed point with 6 bits for fraction, range x[-6.0:6.0], y[-9:9], 2^15/9=3641
		// WARNING swap x and y in remote viewer to align with general x-axis and y-axis usage
		goodEnoughLoc[index].posRos.y = recvBuf.pl.s16[payloadIndex++] / 2048.0; // input is negative top and positive bottom
		goodEnoughLoc[index].posRos.x = recvBuf.pl.s16[payloadIndex++] / 2048.0;
		// use fixed point with 13 bits for fraction, range rz[0:6.28], 2^16/6.28=10436
		// rz = 0 is on y axis
		goodEnoughLoc[index].posRos.rz = recvBuf.pl.u16[payloadIndex++] / 8192.0;

		// use fixed point with 16 bits for fraction, range score[0:1]
		goodEnoughLoc[index].score = recvBuf.pl.u16[payloadIndex++] / 65535.0;
		goodEnoughLoc[index].lastActive = recvBuf.pl.u16[payloadIndex++];
		goodEnoughLoc[index].age = recvBuf.pl.u16[payloadIndex++];
		goodEnoughLoc[index].amountOnFloor = recvBuf.pl.u16[payloadIndex++];
		goodEnoughLoc[index].numberOfTries = recvBuf.pl.u16[payloadIndex++];
		goodEnoughLoc[index].goodEnough = true;

		double x = 0.0;
		double y = 0.0;
		double rz = 0.0;
		posRosHistory[index].erase(posRosHistory[index].begin());
		posRosHistory[index].push_back(goodEnoughLoc[index].posRos);
		for (size_t ii = 0; ii < posRosHistory[index].size(); ii++) {
			x += posRosHistory[index][ii].x;
			y += posRosHistory[index][ii].y;
			rz += posRosHistory[index][ii].rz; // TODO fix for values near 360 degrees
		}
		posRosAverage[index].x = x / posRosHistory[index].size();
		posRosAverage[index].y = y / posRosHistory[index].size();
		posRosAverage[index].rz = rz / posRosHistory[index].size();

#ifdef NONO
		printf(" %10s good %3d %5d : %4.0f %4.0f %3.0f : %3.3f : %3d %3d %3d %3d\n",
				ipAddress, goodEnoughLoc[index].packetCnt, recvBuf.size,
				goodEnoughLoc[index].pos.x, goodEnoughLoc[index].pos.y, goodEnoughLoc[index].pos.rz,
				goodEnoughLoc[index].score,
				goodEnoughLoc[index].lastActive, goodEnoughLoc[index].age,
				goodEnoughLoc[index].amountOnFloor, goodEnoughLoc[index].numberOfTries );
#endif

		fprintf(writeTxt, "%zu %s %10s good %3d %5d : %4.0f %4.0f %3.0f : %3.3f : %3d %3d %3d %3d\n",
				stats[index].remoteTime, stats[index].remoteTimeString, ipAddress, goodEnoughLoc[index].packetCnt,
				recvBuf.size, goodEnoughLoc[index].pos.x, goodEnoughLoc[index].pos.y, goodEnoughLoc[index].pos.rz,
				goodEnoughLoc[index].score, goodEnoughLoc[index].lastActive, goodEnoughLoc[index].age,
				goodEnoughLoc[index].amountOnFloor, goodEnoughLoc[index].numberOfTries);

		goodEnoughLocExportMutex.unlock();
	}
}

void multicastReceive::decodeLinePoints(size_t index) {
	uint8_t objectSize = 2 * 2; // each line point contains 2 values of 2 bytes
	if (((recvBuf.size - HEADER_SIZE) % objectSize) != 0) {
		printf(" %10s : ERROR     : line points packet should be %d + n * %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE, objectSize, recvBuf.size);
	} else {
		linePointListExportMutex.lock();
		linePointList[index].packetCnt = recvBuf.cnt;
		linePointList[index].state = updated;
		linePointList[index].list.clear();

		uint16_t payloadIndex = 0;
		ssize_t amountOfObjects = (recvBuf.size - HEADER_SIZE) / objectSize;
		for (ssize_t ii = 0; ii < amountOfObjects; ii++) {
			ballObstSt point;
			point.size = 0;
			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			point.radius = recvBuf.pl.u16[payloadIndex++] / 2048.0;

			// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
			point.azimuth = recvBuf.pl.u16[payloadIndex++] / 8192.0;
			point.elevation = 0.0;
			linePointList[index].list.push_back(point);
		}
		linePointListExportMutex.unlock();

#ifdef NONO
		printf(" %10s line %3d %5d", ipAddress, linePointList[index].packetCnt, recvBuf.size );
		int maxPrintLine = (recvBuf.size - HEADER_SIZE)/2;
		if( maxPrintLine > 10 ) {maxPrintLine = 10;} // to prevent screen clutter

		for( int ii = 0; ii < maxPrintLine; ii++ ) {
			printf(" : %4.2f %5.1f", linePointList[index].list[ii].radius, linePointList[index].list[ii].angle * 180.0/M_PI);
		}
		printf(" \n" );
#endif

		fprintf(writeTxt, "%zu %s %10s line %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString, ipAddress,
				linePointList[index].packetCnt, recvBuf.size);
		for (int ii = 0; ii < (recvBuf.size - HEADER_SIZE) / 2; ii++) {
			fprintf(writeTxt, " : %4.2f %5.1f", linePointList[index].list[ii].radius,
					linePointList[index].list[ii].azimuth * 180.0 / M_PI);
		}
		fprintf(writeTxt, " \n");
	}
}

void multicastReceive::decodeBallList(size_t index) {
// check if the packet size makes sense
	uint16_t objectSize = 2 * 4; // each ball contains 4 values of 2 bytes
	if (((recvBuf.size - HEADER_SIZE) % objectSize) != 0) {
		printf(" %10s : ERROR     : ball packet should be %d + n * %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE, objectSize, recvBuf.size);
	} else {
		ballListExportMutex.lock();
		ballList[index].packetCnt = recvBuf.cnt;
		ballList[index].state = updated;
		ballList[index].list.clear();

		uint16_t payloadIndex = 0;
		ssize_t amountOfObjects = (recvBuf.size - HEADER_SIZE) / objectSize;
		for (ssize_t ii = 0; ii < amountOfObjects; ii++) {
			ballObstSt point;
			point.size = recvBuf.pl.u16[payloadIndex++];

			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			point.radius = recvBuf.pl.u16[payloadIndex++] / 2048.0;

			// fixed point: 65536/8192 = 8, azimuth range 0 to 6.28 radians
			point.azimuth = recvBuf.pl.u16[payloadIndex++] / 8192.0;

			// fixed point: 65536/8192 = 8, elevation range 0 to 6.28 radians
			point.elevation = recvBuf.pl.u16[payloadIndex++] / 8192.0;
			ballList[index].list.push_back(point);
		}
		ballListExportMutex.unlock();

#ifdef NONO
		printf(" %10s ball %3d %5d", ipAddress, ballList[index].packetCnt, recvBuf.size );
		for( size_t ii = 0; ii < ballList[index].list.size(); ii++ ) {
			printf(" : %4d %4.2f %5.1f %5.1f",
					ballList[index].list[ii].size, ballList[index].list[ii].radius, ballList[index].list[ii].azimuth * 180.0/M_PI,
					ballList[index].list[ii].elevation * 180.0/M_PI );
		}
		printf("\n");
#endif

		fprintf(writeTxt, "%zu %s %10s ball %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString, ipAddress,
				ballList[index].packetCnt, recvBuf.size);
		for (size_t ii = 0; ii < ballList[index].list.size(); ii++) {
			fprintf(writeTxt, " : %4d %4.2f %5.1f %5.1f", ballList[index].list[ii].size,
					ballList[index].list[ii].radius, ballList[index].list[ii].azimuth * 180.0 / M_PI,
					ballList[index].list[ii].elevation * 180.0 / M_PI);
		}
		fprintf(writeTxt, "\n");
	}
}

void multicastReceive::decodeBallFarList(size_t index) {
// check if the packet size makes sense
	uint16_t objectSize = 2 * 4; // each ballFar contains 4 values of 2 bytes
	if (((recvBuf.size - HEADER_SIZE) % objectSize) != 0) {
		printf(" %10s : ERROR     : ballFar packet should be %d + n * %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE, objectSize, recvBuf.size);
	} else {
		ballFarListExportMutex.lock();
		ballFarList[index].packetCnt = recvBuf.cnt;
		ballFarList[index].state = updated;
		ballFarList[index].list.clear();

		uint16_t payloadIndex = 0;
		ssize_t amountOfObjects = (recvBuf.size - HEADER_SIZE) / objectSize;
		for (ssize_t ii = 0; ii < amountOfObjects; ii++) {
			ballObstSt point;
			point.size = recvBuf.pl.u16[payloadIndex++];

			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			point.radius = recvBuf.pl.u16[payloadIndex++] / 2048.0;

			// fixed point: 65536/8192 = 8, azimuth range 0 to 6.28 radians
			point.azimuth = recvBuf.pl.u16[payloadIndex++] / 8192.0;

			// fixed point: 65536/8192 = 8, elevation range 0 to 6.28 radians
			point.elevation = recvBuf.pl.u16[payloadIndex++] / 8192.0;
			ballFarList[index].list.push_back(point);
		}
		ballFarListExportMutex.unlock();

#ifdef NONO
		printf(" %10s farb %3d %5d", ipAddress, ballFarList[index].packetCnt, recvBuf.size );
		for( size_t ii = 0; ii < ballFarList[index].list.size(); ii++ ) {
			printf(" : %4d %4.2f %5.1f %5.1f",
					ballFarList[index].list[ii].size, ballFarList[index].list[ii].radius, ballFarList[index].list[ii].azimuth * 180.0/M_PI,
					ballFarList[index].list[ii].elevation * 180.0/M_PI );
		}
		printf("\n");
#endif

		fprintf(writeTxt, "%zu %s %10s farb %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString, ipAddress,
				ballFarList[index].packetCnt, recvBuf.size);
		for (size_t ii = 0; ii < ballFarList[index].list.size(); ii++) {
			fprintf(writeTxt, " : %4d %4.2f %5.1f %5.1f", ballFarList[index].list[ii].size,
					ballFarList[index].list[ii].radius, ballFarList[index].list[ii].azimuth * 180.0 / M_PI,
					ballFarList[index].list[ii].elevation * 180.0 / M_PI);
		}
		fprintf(writeTxt, "\n");
	}
}

void multicastReceive::decodeCyanList(size_t index) {
// check if the packet size makes sense
	uint16_t objectSize = 2 * 4; // each cyan object contains 4 values of 2 bytes
	if (((recvBuf.size - HEADER_SIZE) % objectSize) != 0) {
		printf(" %10s : ERROR     : cyan packet should be %d + n * %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE, objectSize, recvBuf.size);
	} else {
		cyanListExportMutex.lock();
		cyanList[index].packetCnt = recvBuf.cnt;
		cyanList[index].state = updated;
		cyanList[index].list.clear();

		uint16_t payloadIndex = 0;
		ssize_t amountOfObjects = (recvBuf.size - HEADER_SIZE) / objectSize;
		for (ssize_t ii = 0; ii < amountOfObjects; ii++) {
			ballObstSt point;
			point.size = recvBuf.pl.u16[payloadIndex++];

			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			point.radius = recvBuf.pl.u16[payloadIndex++] / 2048.0;

			// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
			point.azimuth = recvBuf.pl.u16[payloadIndex++] / 8192.0;

			// fixed point: 65536/8192 = 8, elevation range 0 to 6.28 radians
			point.elevation = recvBuf.pl.u16[payloadIndex++] / 8192.0;
			cyanList[index].list.push_back(point);
		}
		cyanListExportMutex.unlock();

#ifdef NONO
		printf(" %10s cyan %3d %5d", ipAddress, cyanList[index].packetCnt, recvBuf.size );
		for( size_t ii = 0; ii < cyanList[index].list.size(); ii++ ) {
			printf(" : %4d %4.2f %5.1f %5.1f",
					cyanList[index].list[ii].size, cyanList[index].list[ii].radius, cyanList[index].list[ii].azimuth * 180.0/M_PI,
					cyanList[index].list[ii].elevation * 180.0/M_PI );
		}
		printf("\n");
#endif

		fprintf(writeTxt, "%zu %s %10s cyan %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString, ipAddress,
				cyanList[index].packetCnt, recvBuf.size);
		for (size_t ii = 0; ii < cyanList[index].list.size(); ii++) {
			fprintf(writeTxt, " : %4d %4.2f %5.1f %5.1f", cyanList[index].list[ii].size,
					cyanList[index].list[ii].radius, cyanList[index].list[ii].azimuth * 180.0 / M_PI,
					cyanList[index].list[ii].elevation * 180.0 / M_PI);
		}
		fprintf(writeTxt, "\n");
	}
}

void multicastReceive::decodeMagentaList(size_t index) {
// check if the packet size makes sense
	uint16_t objectSize = 2 * 4; // each magenta object contains 4 values of 2 bytes
	if (((recvBuf.size - HEADER_SIZE) % objectSize) != 0) {
		printf(" %10s : ERROR     : magenta packet should be %d + n * %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE, objectSize, recvBuf.size);
	} else {
		magentaListExportMutex.lock();
		magentaList[index].packetCnt = recvBuf.cnt;
		magentaList[index].state = updated;
		magentaList[index].list.clear();

		uint16_t payloadIndex = 0;
		ssize_t amountOfObjects = (recvBuf.size - HEADER_SIZE) / objectSize;
		for (ssize_t ii = 0; ii < amountOfObjects; ii++) {
			ballObstSt point;
			point.size = recvBuf.pl.u16[payloadIndex++];

			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			point.radius = recvBuf.pl.u16[payloadIndex++] / 2048.0;

			// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
			point.azimuth = recvBuf.pl.u16[payloadIndex++] / 8192.0;

			// fixed point: 65536/8192 = 8, elevation range 0 to 6.28 radians
			point.elevation = recvBuf.pl.u16[payloadIndex++] / 8192.0;
			magentaList[index].list.push_back(point);
		}
		magentaListExportMutex.unlock();

#ifdef NONO
		printf(" %10s magenta %3d %5d", ipAddress, magentaList[index].packetCnt, recvBuf.size );
		for( size_t ii = 0; ii < magentaList[index].list.size(); ii++ ) {
			printf(" : %4d %4.2f %5.1f %5.1f",
					magentaList[index].list[ii].size, magentaList[index].list[ii].radius, magentaList[index].list[ii].azimuth * 180.0/M_PI,
					magentaList[index].list[ii].elevation * 180.0/M_PI );
		}
		printf("\n");
#endif

		fprintf(writeTxt, "%zu %s %10s magenta %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString,
				ipAddress, magentaList[index].packetCnt, recvBuf.size);
		for (size_t ii = 0; ii < magentaList[index].list.size(); ii++) {
			fprintf(writeTxt, " : %4d %4.2f %5.1f %5.1f", magentaList[index].list[ii].size,
					magentaList[index].list[ii].radius, magentaList[index].list[ii].azimuth * 180.0 / M_PI,
					magentaList[index].list[ii].elevation * 180.0 / M_PI);
		}
		fprintf(writeTxt, "\n");
	}
}

void multicastReceive::decodeObstacleList(size_t index) {
// check if the packet size makes sense
	uint16_t objectSize = 2 * 4; // each obstacle contains 4 values of 2 bytes
	if (((recvBuf.size - HEADER_SIZE) % objectSize) != 0) {
		printf(" %10s : ERROR     : obstacle packet should be %d + n * %d bytes, but received %d bytes\n", ipAddress,
				HEADER_SIZE, objectSize, recvBuf.size);
	} else {
		obstacleListExportMutex.lock();
		obstacleList[index].packetCnt = recvBuf.cnt;
		obstacleList[index].state = updated;
		obstacleList[index].list.clear();

		uint16_t payloadIndex = 0;
		ssize_t amountOfObjects = (recvBuf.size - HEADER_SIZE) / objectSize;
		for (ssize_t ii = 0; ii < amountOfObjects; ii++) {
			ballObstSt point;
			point.size = recvBuf.pl.u16[payloadIndex++];

			// fixed point: 65536/2048 = 32, radius range 0 to 20 meters
			point.radius = recvBuf.pl.u16[payloadIndex++] / 2048.0;

			// fixed point: 65536/8192 = 8, angle range 0 to 6.28 radians
			point.azimuth = recvBuf.pl.u16[payloadIndex++] / 8192.0;

			// fixed point: 65536/8192 = 8, elevation range 0 to 6.28 radians
			point.elevation = recvBuf.pl.u16[payloadIndex++] / 8192.0;
			obstacleList[index].list.push_back(point);
		}
		obstacleListExportMutex.unlock();

#ifdef NONO
		printf(" %10s obst %3d %5d", ipAddress, obstacleList[index].packetCnt, recvBuf.size);
		for( size_t ii = 0; ii < obstacleList[index].list.size(); ii++ ) {
			printf(" : %4d %4.2f %5.1f %5.1f",
					obstacleList[index].list[ii].size, obstacleList[index].list[ii].radius, obstacleList[index].list[ii].azimuth * 180.0/M_PI ,
					obstacleList[index].list[ii].elevation * 180.0/M_PI );
		}
		printf("\n");
#endif

		fprintf(writeTxt, "%zu %s %10s obst %3d %5d", stats[index].remoteTime, stats[index].remoteTimeString, ipAddress,
				obstacleList[index].packetCnt, recvBuf.size);
		for (size_t ii = 0; ii < obstacleList[index].list.size(); ii++) {
			fprintf(writeTxt, " : %4d %4.2f %5.1f %5.1f", obstacleList[index].list[ii].size,
					obstacleList[index].list[ii].radius, obstacleList[index].list[ii].azimuth * 180.0 / M_PI,
					obstacleList[index].list[ii].elevation * 180.0 / M_PI);
		}
		fprintf(writeTxt, "\n");
	}
}

void multicastReceive::receive() {
	while (1) {
		size_t index = MAX_ROBOTS;
		bool readFromSocket = true;

#ifdef READFROMFILE
		readFromSocket = false;

		// read the packets from file (available in the packets vector struct)
		uint8_t *readPtr; // use pointer to iterate through the receiver buffer
		readPtr = &recvBuf.type; // assign the address of the first element of the receive buffer to the pointer

		// wrap around the packet index when we reach the end (and prevent reading out of range)
		if ((packetIndex + 1) > packets.size()) {
			packetIndex = 0; // wrap around
		}

		index = packets[packetIndex].index;
		sprintf(ipAddress, "robot %zu", index + 1);

		for (size_t jj = 0; jj < packets[packetIndex].data.size(); jj++) {
			*readPtr = packets[packetIndex].data[jj];
			readPtr++;
		}

		nbytes = packets[packetIndex].data.size();

		// read the next packet for the next iteration
		if (!packetIndexPause) {
			packetIndex++;
		}

		usleep(5000);

#else
		// block until data received
		if ((nbytes = recvfrom(fd, &recvBuf, sizeof(recvBuf), 0, &fromAddr, &fromAddrLen)) < 0) {
			perror("ERROR     : cannot receive data, message");
			exit(EXIT_FAILURE);
		}

		// extract the ip address from from fromAddr struct
		inet_ntop(AF_INET, &(((struct sockaddr_in *) &fromAddr)->sin_addr), ipAddress, sizeof(ipAddress));

		// determine from which robot the packet was comming
		if (strcmp(ipAddress, "172.16.74.51") == 0) {
			index = 0;
		} // robot 1
		else if (strcmp(ipAddress, "172.16.74.52") == 0) {
			index = 1;
		} // robot 2
		else if (strcmp(ipAddress, "172.16.74.53") == 0) {
			index = 2;
		} // robot 3
		else if (strcmp(ipAddress, "172.16.74.54") == 0) {
			index = 3;
		} // robot 4
		else if (strcmp(ipAddress, "172.16.74.55") == 0) {
			index = 4;
		} // robot 5
		else if (strcmp(ipAddress, "172.16.74.56") == 0) {
			index = 5;
		} // robot 6
		else if (strcmp(ipAddress, "172.16.74.57") == 0) {
			index = 6;
		} // robot 7
		else if (strcmp(ipAddress, "172.16.74.58") == 0) {
			index = 7;
		} // robot 8
		else if (strcmp(ipAddress, "172.16.74.59") == 0) {
			index = 8;
		} // robot 9
		else if (strcmp(ipAddress, "10.0.0.1") == 0) {
			index = 2;
		} // tweety acts as robot 2
		else if (strcmp(ipAddress, "10.0.0.61") == 0) {
			index = 4;
		} // pannekoek acts as robot 5
		else if (strcmp(ipAddress, "10.0.0.64") == 0) {
			index = 3;
		} // fiction acts as robot 4
		else if (strcmp(ipAddress, "10.0.0.65") == 0) {
			index = 3;
		} // fiction acts as robot 4
		else if (strcmp(ipAddress, "10.0.0.68") == 0) {
			index = 3;
		} // fiction acts as robot 4
		else if (strcmp(ipAddress, "10.0.0.129") == 0) {
			index = 7;
		} // fiction acts as robot 8
		else if (strncmp(ipAddress, "172.16.74.1", 11) == 0) {
			index = 3;
		} // fiction acts as robot 4
		else if (strncmp(ipAddress, "172.16.74.2", 11) == 0) {
			index = 3;
		} // fiction acts as robot 4
		else if (strncmp(ipAddress, "192.168.", 8) == 0) {
			index = 3;
		} // fiction acts as robot 4
		else if (strcmp(ipAddress, "10.116.82.137") == 0) {
			index = 0;
		} // robot 1
		else {
			printf(" %10s : ERROR     : do not know how to decode ip address %s\n", ipAddress, ipAddress);
			exit(EXIT_FAILURE);
		}
#endif

		// fwrite(&recvBuf, nbytes, 1, writeBin);
		// do not know how to extract the different packets from the file created with fwrite, so just use ascii to print the numbers and use \n as separator
		fprintf(writeBin, "%02x", (uint8_t) index); // first print the robot index to the logfile
		uint8_t *writePtr; // use pointer to iterate through the receiver buffer
		writePtr = &recvBuf.type; // assign the address of the first element of the receive buffer to the pointer
		for (int ii = 0; ii < nbytes; ii++) {
			fprintf(writeBin, "%02x", *(writePtr++));
		}
		fprintf(writeBin, "\n");

		if (index >= MAX_ROBOTS) {
			printf(" %10s : ERROR     : index %lu out of range (0 to %d)\n", ipAddress, index, MAX_ROBOTS);
		}

		if (readFromSocket || (!packetIndexPause)) {
			packetCnt[index] = packetCnt[index] % 256;
			if (recvBuf.cnt != packetCnt[index]) {
				// printf(" %10s : ERROR     : expected packet counter %d, but got %d\n", ipAddress, packetCnt[index], recvBuf.cnt);
				packetErrorCnt[index]++;
				packetCnt[index] = recvBuf.cnt + 1;
			} else {
				packetCnt[index]++;
			}
			packetCnt[index] = packetCnt[index] % 256;
		}

		if (recvBuf.size != nbytes) {
			printf(" %10s : ERROR     : expected %d bytes, but got %d bytes\n", ipAddress, recvBuf.size, nbytes);
		}

		if (readFromSocket || (packetIndexPrev != packetIndex)) {
			// only update console and viewer if new packet
			if (recvBuf.type == TYPE_STATS) {
				decodeStats(index);
			} else if (recvBuf.type == TYPE_LINEPOINTS) {
				decodeLinePoints(index);
			} else if (recvBuf.type == TYPE_LOCALIZATION) {
				decodeLocalization(index);
			} else if (recvBuf.type == TYPE_GOOD_ENOUGH_LOC) {
				decodeGoodEnoughLoc(index);
			} else if (recvBuf.type == TYPE_BALLDETECTION) {
				decodeBallList(index);
			} else if (recvBuf.type == TYPE_BALLFARDETECTION) {
				decodeBallFarList(index);
			} else if (recvBuf.type == TYPE_OBSTACLES) {
				decodeObstacleList(index);
			} else if (recvBuf.type == TYPE_CYANDETECTION) {
				decodeCyanList(index);
			} else if (recvBuf.type == TYPE_MAGENTADETECTION) {
				decodeMagentaList(index);
			} else {
				printf(" %10s : ERROR     : received unexpected type %d\n", ipAddress, recvBuf.type);
			}
		}

		// store the current packetIndex for the next iteration to be able to check we need to update the viewer and logging in console
		packetIndexPrev = packetIndex;
	}
}

size_t multicastReceive::getPacketErrorCnt(size_t index) {
	statsExportMutex.lock();
	size_t retVal = packetErrorCnt[index];
	statsExportMutex.unlock();
	return retVal;
}

statsSt multicastReceive::getStats(size_t index) {
	statsExportMutex.lock();
	statsSt retVal = stats[index];
	statsExportMutex.unlock();
	return retVal;
}

std::vector<statsSt> multicastReceive::getStatsSort() {
	std::vector<statsSt> statsSort;
	statsExportMutex.lock();
	statsSort = stats;
	statsExportMutex.unlock();
	sort(statsSort.begin(), statsSort.end());
	return statsSort;
}

ballObstListSt multicastReceive::getLinePointList(size_t index) {
	linePointListExportMutex.lock();
	ballObstListSt retVal = linePointList[index];
	linePointListExportMutex.unlock();
	return retVal;
}

goodEnoughLocSt multicastReceive::getGoodEnoughLoc(size_t index) {
	goodEnoughLocExportMutex.lock();
	goodEnoughLocSt retVal = goodEnoughLoc[index];
	goodEnoughLocExportMutex.unlock();
	return retVal;
}

positionStDbl multicastReceive::getGoodEnoughLocRosAverage(size_t index) {
	goodEnoughLocExportMutex.lock();
	positionStDbl retVal = posRosAverage[index];
	goodEnoughLocExportMutex.unlock();
	return retVal;
}

locListSt multicastReceive::getLocList(size_t index) {
	locListExportMutex.lock();
	locListSt retVal = locList[index];
	locListExportMutex.unlock();
	return retVal;
}

ballObstListSt multicastReceive::getBallList(size_t index, size_t type) {
	ballObstListSt retVal;
	if (type == TYPE_BALLDETECTION) {
		ballListExportMutex.lock();
		retVal = ballList[index];
		ballListExportMutex.unlock();
	} else if (type == TYPE_BALLFARDETECTION) {
		ballFarListExportMutex.lock();
		retVal = ballFarList[index];
		ballFarListExportMutex.unlock();
	} else if (type == TYPE_CYANDETECTION) {
		cyanListExportMutex.lock();
		retVal = cyanList[index];
		cyanListExportMutex.unlock();
	} else if (type == TYPE_MAGENTADETECTION) {
		magentaListExportMutex.lock();
		retVal = magentaList[index];
		magentaListExportMutex.unlock();
	} else {
		printf(" %10s : ERROR     : type %zu out of range\n", ipAddress, type);
		exit(EXIT_FAILURE);
	}
	return retVal;
}

ballObstListSt multicastReceive::getObstacleList(size_t index) {
	obstacleListExportMutex.lock();
	ballObstListSt retVal = obstacleList[index];
	obstacleListExportMutex.unlock();
	return retVal;
}

void multicastReceive::packetIndexAdd(int value) {
	int newIndex = packetIndex + value;
	if (newIndex < 0) {
		// so the value was negative, wrap from beginning of index to end of index
		newIndex = packets.size() + newIndex;
	} else if ((newIndex + 1) > (int) packets.size()) {
		// so the value was positive and we where already at the end of the index, wrap to the beginning of the index
		newIndex = newIndex + 1 - packets.size();
	}
	packetIndex = (size_t) newIndex;
// printf(" %10s : INFO      : value change %d results to new packetIndex %zu\n", ipAddress, value, packetIndex);
}

hhMmSsSt multicastReceive::secondsToHhMmSs(uint16_t input) {
	hhMmSsSt retVal;
	if (input == 0xffff) {
		// make clear the provided time is out of range
		retVal.hours = 99;
		retVal.minutes = 99;
		retVal.seconds = 99;

	} else {
		retVal.hours = input / 3600;
		retVal.minutes = (input - retVal.hours * 3600) / 60;
		retVal.seconds = input - retVal.hours * 3600 - retVal.minutes * 60;
	}
	return retVal;
}
