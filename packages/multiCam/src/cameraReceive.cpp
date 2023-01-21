// Copyright 2018-2021 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef NOROS
#include "tracing.hpp"
#endif

#include "cameraReceive.hpp"

#include <errno.h>
#include <stdio.h>
#include <string.h>

using namespace cv;
using namespace std;

bool ballPointListValid[4];
bool ballFarPointListValid[4];
bool obstaclePointListValid[4];
size_t linePointListLongAxisAge[4];
size_t linePointListShortAxisAge[4];
bool floorPointListValid[4];

cameraReceive::cameraReceive(configurator *conf) {
	this->conf = conf;

	// ## network ##
	// create a normal UDP socket
	if ((fd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cannot create UDP socket, %s\n", strerror(errno));
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

	// fill in the local address structure
	struct sockaddr_in localAddr;
	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(44444); // data port
	// bind to the local address / port
	if (::bind(fd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		printf("ERROR     : cannot bind to UDP socket, %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset(&mreq, 0, sizeof(mreq)); // set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	// first try IP robot configuration
	mreq.imr_interface.s_addr = inet_addr("10.0.0.1");
	if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
		// fallback
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
			printf("ERROR     : cannot join the multicast group, check if route exist, message %s\n", strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
	}

	printf("INFO      : camera receive multicast group address %s port %u\n", inet_ntoa(mreq.imr_multiaddr),
			ntohs(localAddr.sin_port));
	// ## network done ##
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double localTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
	deltaTimePrintCounter = 0;
	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		rxPacketCntExpected[camIndex] = 0; // initialize at 0
		rxPacketCntFirstCheck[camIndex] = true;
		receiveTime[camIndex] = localTime;
		ballPointListValid[camIndex] = false;
		ballFarPointListValid[camIndex] = false;
		obstaclePointListValid[camIndex] = false;
		linePointListLongAxisAge[camIndex] = 1000; // TODO set max size
		linePointListShortAxisAge[camIndex] = 1000; // TODO set max size
		floorPointListValid[camIndex] = false;
		ballReceiveTime[camIndex] = localTime;
	}

	statisticsExportMutex.lock();
	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		receiveTimeDelta[camIndex] = 0;
	}
	statisticsExportMutex.unlock();

	linePointListLongAxisExportMutex.unlock(); // probably not needed
	linePointListShortAxisExportMutex.unlock(); // probably not needed
	floorPointListExportMutex.unlock(); // probably not needed
	obstaclePointListExportMutex.unlock(); // probably not needed
	cameraFrameExportMutex.lock();
	for (size_t ii = 0; ii < 4; ii++) {
		cameraFrame[ii] = Mat::zeros(SEND_IMAGE_HEIGHT, SEND_IMAGE_WIDTH, CV_8UC3);
	}
	cameraFrameExportMutex.unlock();

	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		anaFrameCounter[camIndex] = 0;
		anaApplUptime[camIndex] = 0;
	}

	waitForData = true;
}

void cameraReceive::block() {
#ifndef NOROS
	TRACE_FUNCTION("Waiting for data...");
#endif
	while (waitForData) {
		usleep(100);
	}
	waitForData = true;
}

void cameraReceive::receive() {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	while (true) {
		// block until data received
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		ssize_t nBytes = recvfrom(fd, &rxPacket, sizeof(camPacketT), 0, (struct sockaddr *) &fromAddr, &fromAddrlen);
		if (nBytes < 0) {
			perror("ERROR     : reading from socket, message");
			close(fd);
			exit(EXIT_FAILURE);
		}

		bool valid = true;
		if (rxPacket.size != nBytes) {
			printf("ERROR     : received %zd bytes, but expected %u bytes\n", nBytes, rxPacket.size);
			valid = false;
		} else {
			// printf("INFO      : received %u bytes from sender %s:%u\n",
			//        rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//        ntohs(fromAddr.sin_port));
		}

		size_t camIndex = rxPacket.id >> 6; // highest 2 bits contain the localization of the camera

		if (valid && (rxPacket.cnt != rxPacketCntExpected[camIndex])) {
			if (rxPacketCntFirstCheck[camIndex]) {
				rxPacketCntFirstCheck[camIndex] = false;
				rxPacketCntExpected[camIndex] = rxPacket.cnt + 1; // set the expected for the next received camera packet
			} else {
				int delta = (256 + rxPacket.cnt - rxPacketCntExpected[camIndex]) % 256;
				if (delta > 4) {
					printf(
							"WARNING   : cam %zu received camera packet counter %3u, but expected packet counter %3u, which already occurred for the %d packet\n",
							camIndex, rxPacket.cnt, rxPacketCntExpected[camIndex], delta);
				}
				rxPacketCntExpected[camIndex] = rxPacket.cnt + 1;
				valid = false;
			}
		} else {
			rxPacketCntExpected[camIndex]++;
		}

		if (valid) {
			// lowest 6 bits contain the packet type
			if ((rxPacket.id & 0x3f) == 1) { // ## statistics ##
				size_t payloadSize = rxPacket.size - CAM_PACKET_HEADER_SIZE;
				if (payloadSize != 7) {
					printf("ERROR     : cam %zu expected a payload of 8 bytes but got %zu for statistics packet\n",
							camIndex, payloadSize);
				} else {
					anaFrameCounter[camIndex] = rxPacket.pl.u32[0 / 4];
					anaApplUptime[camIndex] = rxPacket.pl.u16[4 / 2]; // application uptime in seconds, be careful, index jump
					camValAverage[camIndex] = rxPacket.pl.u8[6 / 1]; // average of camera value
					// printf("INFO      : cam %zu frame counter %10d ana appl uptime %10d seconds camera average value %u\n", camIndex, anaFrameCounter[camIndex], anaApplUptime[camIndex], camValAverage[camIndex]);

#ifdef NONO
					gettimeofday(&tv, NULL);
					receiveTime[camIndex] = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
					if (camIndex == 0) {
						if (deltaTimePrintCounter == 2) {
							double msDeltaCam1 = receiveTime[0] - receiveTime[1];
							double msDeltaCam2 = receiveTime[0] - receiveTime[2];
							double msDeltaCam3 = receiveTime[0] - receiveTime[3];
							printf(
									"INFO      : delta receive time from cam0 to cam1 %5.1f ms cam2 %5.1f ms cam3 %5.1f ms\n",
									1000 * msDeltaCam1, 1000 * msDeltaCam2, 1000 * msDeltaCam3);
							deltaTimePrintCounter = 0;
						}
						deltaTimePrintCounter++;
					}
#else

					struct timeval tv;
					gettimeofday(&tv, NULL);
					receiveTime[camIndex] = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
					if (camIndex == 0) {
						statisticsExportMutex.lock();
						for (size_t ii = 1; ii < 4; ii++) {
							// calculate time different cam0 and other camera's
							receiveTimeDelta[ii] = receiveTime[0] - receiveTime[ii];
						}
						statisticsExportMutex.unlock();
						if (false & (deltaTimePrintCounter == 2)) {
							printf(
									"INFO      : delta receive time from cam0 to cam1 %5.1f ms cam2 %5.1f ms cam3 %5.1f ms\n",
									1000.0 * receiveTimeDelta[1], 1000.0 * receiveTimeDelta[2],
									1000.0 * receiveTimeDelta[3]);
							deltaTimePrintCounter = 0;
						}
						deltaTimePrintCounter++;
					}
#endif
				}

			} else if ((rxPacket.id & 0x3f) == 2) { // ## line points from long axis camera ##
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::line_points_from_long_axis_camera", "");
#endif
				size_t points = (rxPacket.size - CAM_PACKET_HEADER_SIZE) / (2 * 4); // see sender to determine how many bytes per point
				{
					lock_guard < mutex > lineMutex(linePointListLongAxisExportMutex); // lock the mutex
					linePointListLongAxis[camIndex].clear();
					for (size_t ii = 0; ii < points; ii++) {
						linePointSt linePoint;
						linePoint.xBegin = rxPacket.pl.u16[4 * ii];
						linePoint.xEnd = rxPacket.pl.u16[4 * ii + 1];
						linePoint.yBegin = rxPacket.pl.u16[4 * ii + 2];
						linePoint.yEnd = rxPacket.pl.u16[4 * ii + 3];
						linePoint.size = 0; // not provided anymore
						if ((linePoint.xBegin > linePoint.xEnd) || (linePoint.xBegin > (ROI_WIDTH - 1))
								|| (linePoint.xEnd > (ROI_WIDTH - 1)) || (linePoint.yBegin > ( ROI_HEIGHT - 1))
								|| (linePoint.yEnd > ( ROI_HEIGHT - 1))) {
							printf(
									"ERROR     : cam %zu receive out of range xBegin %3u xEnd %3u yBegin %3u yEnd %3u for long axis line points\n",
									camIndex, linePoint.xBegin, linePoint.xEnd, linePoint.yBegin, linePoint.yEnd);
						} else {
							linePointListLongAxis[camIndex].push_back(linePoint);
						}
					}
					if (camIndex == 0) {
						// wait for line points from camera 1 to start the localization
						// because the FPGA boards are currently running asynchronous there is an issue when the robot's are moving
						// to prevent snake balls synchronize on camera 0
						waitForData = false;
					}
					linePointListLongAxisAge[camIndex] = 0; // used to determine if line points are valid and can be exported to line point detection
				}
			} else if ((rxPacket.id & 0x3f) == 6) { // ## line points from short axis camera ##
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::line_points_from_short_axis_camera", "");
#endif
				size_t points = (rxPacket.size - CAM_PACKET_HEADER_SIZE) / (2 * 4); // see sender to determine how many bytes per point
				{
					lock_guard < mutex > lineMutex(linePointListShortAxisExportMutex); // lock the mutex
					linePointListShortAxis[camIndex].clear();
					for (size_t ii = 0; ii < points; ii++) {
						linePointSt linePoint;
						linePoint.xBegin = rxPacket.pl.u16[4 * ii];
						linePoint.xEnd = rxPacket.pl.u16[4 * ii + 1];
						linePoint.yBegin = rxPacket.pl.u16[4 * ii + 2];
						linePoint.yEnd = rxPacket.pl.u16[4 * ii + 3];
						linePoint.size = 0; // not provided anymore
						if ((linePoint.xBegin > linePoint.xEnd) || (linePoint.xBegin > (ROI_WIDTH - 1))
								|| (linePoint.xEnd > (ROI_WIDTH - 1)) || (linePoint.yBegin > ( ROI_HEIGHT - 1))
								|| (linePoint.yEnd > ( ROI_HEIGHT - 1))) {
							printf(
									"ERROR     : cam %zu receive out of range xBegin %3u xEnd %3u yBegin %3u yEnd %3u for short axis line points\n",
									camIndex, linePoint.xBegin, linePoint.xEnd, linePoint.yBegin, linePoint.yEnd);
						} else {
							linePointListShortAxis[camIndex].push_back(linePoint);
						}
					}
					linePointListShortAxisAge[camIndex] = 0; // used to determine if line points are valid and can be exported to line point detection
				}
			} else if ((rxPacket.id & 0x20) == 0x20) { // bit 5 is used to identify as camera frame packet = image
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::camera_frame_packet", "");
#endif
				// the image does not fit in one packet, receive in multiple packets, use id to inform receiver
				// which part of the image it received
				uint8_t imagePart = rxPacket.id & 0x1f; // maximal size 32 parts
				size_t imageSize = sizeof(cameraFrameRecv[camIndex]);
				if (imagePart == 0) {
					// printf("INFO      : new image\n");
					memset(&cameraFrameRecv[camIndex], 0, imageSize);
				}
				size_t bufferIndex = imagePart * CAM_PACKET_PAYLOAD_SIZE;
				size_t payloadSize = rxPacket.size - CAM_PACKET_HEADER_SIZE;
				size_t receivedAlready = bufferIndex + payloadSize;
				if (receivedAlready > imageSize) {
					printf(
							"ERROR     : cam %zu total received %zu bytes which does not camera frame receive buffer %zu bytes\n",
							camIndex, receivedAlready, imageSize);
					fflush(stdout);
					exit(EXIT_FAILURE);
				}
				//printf(
				//        "INFO      : image part %2u buffer index %7zu payload size %zu received already %zu image size %zu\n",
				//        imagePart, bufferIndex, payloadSize, receivedAlready, imageSize);
				memcpy(&cameraFrameRecv[camIndex][bufferIndex], &rxPacket.pl, payloadSize);
				if (receivedAlready == imageSize) {
					// printf("INFO      : image complete\n");
					fflush(stdout);
					cameraFrameExportMutex.lock();
					size_t bufferIndex = 0;
					for (size_t yy = 0; yy < SEND_IMAGE_HEIGHT; yy++) { // lines
						for (size_t xx = 0; xx < SEND_IMAGE_WIDTH; xx++) { // pixels
							cameraFrame[camIndex].at<Vec3b>(yy, xx)[2] = cameraFrameRecv[camIndex][bufferIndex++]; // red
							cameraFrame[camIndex].at<Vec3b>(yy, xx)[1] = cameraFrameRecv[camIndex][bufferIndex++]; // green
							cameraFrame[camIndex].at<Vec3b>(yy, xx)[0] = cameraFrameRecv[camIndex][bufferIndex++]; // blue
						}
					}
					cameraFrameExportMutex.unlock();
				}

			} else if ((rxPacket.id & 0x3f) == 3) { // ## ball ##
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::ball", "");
#endif
				struct timeval tv;
				gettimeofday(&tv, NULL);
				ballReceiveTime[camIndex] = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;

				size_t points = (rxPacket.size - CAM_PACKET_HEADER_SIZE) / (2 * 4); // see sender to determine how many bytes per point
				// printf("INFO      : cam %zu received %zu ball points\n", camIndex, points);
				{
					lock_guard < mutex > ballMutex(ballPointListExportMutex); // lock the mutex
					ballPointList[camIndex].clear();
					for (size_t ii = 0; ii < points; ii++) {
						linePointSt point;
						point.xBegin = rxPacket.pl.u16[4 * ii];
						point.xEnd = rxPacket.pl.u16[4 * ii + 1];
						point.yBegin = rxPacket.pl.u16[4 * ii + 2];
						point.yEnd = rxPacket.pl.u16[4 * ii + 3];
						point.size = 0; // not provided anymore
						if ((point.xBegin > point.xEnd) || (point.xBegin > (ROI_WIDTH - 1))
								|| (point.xEnd > (ROI_WIDTH - 1)) || (point.yBegin > ( ROI_HEIGHT - 1))
								|| (point.yEnd > ( ROI_HEIGHT - 1))) {
							printf(
									"ERROR     : cam %zu receive data out of range xBegin %3u xEnd %3u yBegin %3u yEnd %3u for ball points\n",
									camIndex, point.xBegin, point.xEnd, point.yBegin, point.yEnd);
						} else {
							ballPointList[camIndex].push_back(point);
						}
					}
					ballPointListValid[camIndex] = true; // used to detect spurious wakeup in the receiving thread
				}
				ballDetCondVar.notify_all(); // inform all 4 waiting threads new data is available (only the one with ballPointListValid = true will continue)

			} else if ((rxPacket.id & 0x3f) == 7) { // ## ballFar ##
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::ball_far", "");
#endif
				size_t points = (rxPacket.size - CAM_PACKET_HEADER_SIZE) / (2 * 4); // see sender to determine how many bytes per point
				if (points > 0) {
					printf("INFO      : cam %zu received %zu ballFar points\n", camIndex, points);
				}
				{
					lock_guard < mutex > ballFarMutex(ballFarPointListExportMutex); // lock the mutex
					ballFarPointList[camIndex].clear();
					for (size_t ii = 0; ii < points; ii++) {
						linePointSt point;
						point.xBegin = rxPacket.pl.u16[4 * ii];
						point.xEnd = rxPacket.pl.u16[4 * ii + 1];
						point.yBegin = rxPacket.pl.u16[4 * ii + 2];
						point.yEnd = rxPacket.pl.u16[4 * ii + 3];
						point.size = 0; // not provided anymore
						if ((point.xBegin > point.xEnd) || (point.xBegin > (ROI_WIDTH - 1))
								|| (point.xEnd > (ROI_WIDTH - 1)) || (point.yBegin > ( ROI_HEIGHT - 1))
								|| (point.yEnd > ( ROI_HEIGHT - 1))) {
							printf(
									"ERROR     : cam %zu receive data range xBegin %3u xEnd %3u yBegin %3u yEnd %3u for ballFar points\n",
									camIndex, point.xBegin, point.xEnd, point.yBegin, point.yEnd);
						} else {
							ballFarPointList[camIndex].push_back(point);
						}
					}
					ballFarPointListValid[camIndex] = true; // used to detect spurious wakeup in the receiving thread
				}
				ballFarDetCondVar.notify_all(); // inform all 4 waiting threads new data is available (only the one with ballFarPointListValid = true will continue)

			} else if ((rxPacket.id & 0x3f) == 5) { // ## floor ##
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::floor", "");
#endif
				size_t points = (rxPacket.size - CAM_PACKET_HEADER_SIZE) / (2 * 4); // see sender to determine how many bytes per point
				// printf("INFO      : received %zu floor points\n", points);
				{
					lock_guard < mutex > floorMutex(floorPointListExportMutex); // lock the mutex
					floorPointList[camIndex].clear();
					for (size_t ii = 0; ii < points; ii++) {
						linePointSt point;
						point.xBegin = rxPacket.pl.u16[4 * ii];
						point.xEnd = rxPacket.pl.u16[4 * ii + 1];
						point.yBegin = rxPacket.pl.u16[4 * ii + 2];
						point.yEnd = rxPacket.pl.u16[4 * ii + 3];
						point.size = 0; // not provided anymore
						if ((point.xBegin > point.xEnd) || (point.xBegin > ((ROI_WIDTH / 2) - 1))
								|| (point.xEnd > ((ROI_WIDTH / 2) - 1)) || (point.yBegin > ( ROI_HEIGHT - 1))
								|| (point.yEnd > ( ROI_HEIGHT - 1))) {
							printf(
									"ERROR     : receiving for camera %zu out of range xBegin %3u xEnd %3u yBegin %3u yEnd %3u for floor points\n",
									camIndex, point.xBegin, point.xEnd, point.yBegin, point.yEnd);
						} else {
							floorPointList[camIndex].push_back(point);
						}
					}
					floorPointListValid[camIndex] = true; // used to detect spurious wakeup in the receiving thread
				}

			} else if ((rxPacket.id & 0x3f) == 4) { // ## obstacle ##
#ifndef NOROS
				TRACE_SCOPE("cameraReceive::obstacle", "");
#endif
				size_t points = (rxPacket.size - CAM_PACKET_HEADER_SIZE) / (2 * 4); // see sender to determine how many bytes per point
				{
					lock_guard < mutex > obstacleMutex(obstaclePointListExportMutex); // lock the mutex
					obstaclePointList[camIndex].clear();
					for (size_t ii = 0; ii < points; ii++) {
						linePointSt point;
						point.xBegin = rxPacket.pl.u16[4 * ii];
						point.xEnd = rxPacket.pl.u16[4 * ii + 1];
						point.yBegin = rxPacket.pl.u16[4 * ii + 2];
						point.yEnd = rxPacket.pl.u16[4 * ii + 3];
						point.size = 0; // not provided anymore
						if ((point.xBegin > point.xEnd) || ((point.xBegin > ((ROI_WIDTH / 2) - 1)))
								|| (point.xEnd > ((ROI_WIDTH / 2) - 1)) || (point.yBegin > ( ROI_HEIGHT - 1))
								|| (point.yEnd > ( ROI_HEIGHT - 1))) {
							printf(
									"ERROR     : receiving for camera %zu out of range xBegin %3u xEnd %3u yBegin %3u yEnd %3u for obstacle points\n",
									camIndex, point.xBegin, point.xEnd, point.yBegin, point.yEnd);
						} else {
							obstaclePointList[camIndex].push_back(point);
						}
					}
					obstaclePointListValid[camIndex] = true; // used to detect spurious wakeup in the receiving thread
				}
				obstacleDetCondVar.notify_all();

			} else {
				printf("WARNING   : cam %zu received unknown camera packet id %u\n", camIndex, rxPacket.id);
			}
		}
	}
}

size_t cameraReceive::getLinePointsLongAxisAmount(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	if (camIndex >= 4) {
		printf(
				"ERROR     : tried to get the amount long axis line points from camera index %zu, while the index goes to 3",
				camIndex);
		exit(EXIT_FAILURE);
	}
	std::unique_lock < std::mutex > myLock(linePointListLongAxisExportMutex);
	size_t amount = linePointListLongAxis[camIndex].size();
	myLock.unlock();
	return amount;
}

size_t cameraReceive::getLinePointsShortAxisAmount(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	if (camIndex >= 4) {
		printf(
				"ERROR     : tried to get the amount short axix line points from camera index %zu, while the index goes to 3",
				camIndex);
		exit(EXIT_FAILURE);
	}
	std::unique_lock < std::mutex > myLock(linePointListShortAxisExportMutex);
	size_t amount = linePointListShortAxis[camIndex].size();
	myLock.unlock();
	return amount;
}

vector<linePointSt> cameraReceive::getLinePointsLongAxis(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	if (camIndex >= 4) {
		printf("ERROR     : tried to get long axis line points from camera index %zu, while the index goes to 3",
				camIndex);
		exit(EXIT_FAILURE);
	}

	// acquire mutex used to protect the linePointListLongAxis and linePointListLongAxisAge
	std::unique_lock < std::mutex > myLock(linePointListLongAxisExportMutex);
	vector<linePointSt> retVal;
	// reject if line point data is to old (e.g. because the camera fails)
	// for each camera this call should be performed once per frame
	// because of jitter it might be called 2 times, but 3 times should not happen
	// then no line points or camera down
	if (linePointListLongAxisAge[camIndex] < 2) {
		// copy data to new vector, so the mutex can be released and vector can be changed by the receiver thread
		retVal = linePointListLongAxis[camIndex];
	} else {
		// next print used to determine how often we miss the line points for the current localization (set above threshold to 1)
		// printf("WARNING   : cam %zu no long  axis line points available\n", camIndex);
	}
	linePointListLongAxisAge[camIndex]++; // flag used data is older
	myLock.unlock();

	return retVal;
}

vector<linePointSt> cameraReceive::getLinePointsShortAxis(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	if (camIndex >= 4) {
		printf("ERROR     : tried to get short axis line points from camera index %zu, while the index goes to 3",
				camIndex);
		exit(EXIT_FAILURE);
	}

	// acquire mutex used to protect the linePointListShortAxis and linePointListShortAxisAge
	std::unique_lock < std::mutex > myLock(linePointListShortAxisExportMutex);
	vector<linePointSt> retVal;
	// reject if line point data is to old (e.g. because the camera fails)
	// for each camera this call should be performed once per frame
	// because of jitter it might be called 2 times, but 3 times should not happen
	// then no line points or camera down
	if (linePointListShortAxisAge[camIndex] < 2) {
		// copy data to new vector, so the mutex can be released and vector can be changed by the receiver thread
		retVal = linePointListShortAxis[camIndex];
	} else {
		// next print used to determine how often we miss the line points for the current localization (set above threshold to 1)
		// printf("WARNING   : cam %zu no short axis line points available\n", camIndex);
	}
	linePointListShortAxisAge[camIndex]++; // flag used data is older
	myLock.unlock();

	return retVal;
}

vector<linePointSt> cameraReceive::getAndClearBallPointsWait(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	// acquire mutex used to protect the ballPointList and ballPointListValid
	std::unique_lock < std::mutex > myLock(ballPointListExportMutex);
	// wait for data
	if (camIndex == 0) {
		ballDetCondVar.wait(myLock, [] {return ballPointListValid[0];});
	} else if (camIndex == 1) {
		ballDetCondVar.wait(myLock, [] {return ballPointListValid[1];});
	} else if (camIndex == 2) {
		ballDetCondVar.wait(myLock, [] {return ballPointListValid[2];});
	} else if (camIndex == 3) {
		ballDetCondVar.wait(myLock, [] {return ballPointListValid[3];});
	} else {
		printf("ERROR     : tried to get ball points from camera index %zu, while the index goes to 3", camIndex);
		exit(EXIT_FAILURE);
	}

	// now the new list is available and the mutex is locked
	ballPointListValid[camIndex] = false; // wait again in next iteration until new data is provided

	// copy data to new vector, so the mutex can be released and vector can be changed by the receiver thread
	vector<linePointSt> retVal = ballPointList[camIndex];
	myLock.unlock();

	return retVal;
}

vector<linePointSt> cameraReceive::getAndClearBallFarPointsWait(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	// acquire mutex used to protect the ballFarPointList and ballFarPointListValid
	std::unique_lock < std::mutex > myLock(ballFarPointListExportMutex);
	// wait for data
	if (camIndex == 0) {
		ballFarDetCondVar.wait(myLock, [] {return ballFarPointListValid[0];});
	} else if (camIndex == 1) {
		ballFarDetCondVar.wait(myLock, [] {return ballFarPointListValid[1];});
	} else if (camIndex == 2) {
		ballFarDetCondVar.wait(myLock, [] {return ballFarPointListValid[2];});
	} else if (camIndex == 3) {
		ballFarDetCondVar.wait(myLock, [] {return ballFarPointListValid[3];});
	} else {
		printf("ERROR     : tried to get ballFar points from camera index %zu, while the index goes to 3", camIndex);
		exit(EXIT_FAILURE);
	}

	// now the new list is available and the mutex is locked
	ballFarPointListValid[camIndex] = false; // wait again in next iteration until new data is provided

	// copy data to new vector, so the mutex can be released and vector can be changed by the receiver thread
	vector<linePointSt> retVal = ballFarPointList[camIndex];
	myLock.unlock();

	return retVal;
}

vector<linePointSt> cameraReceive::getFloorPoints(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	if (camIndex >= 4) {
		printf("ERROR     : tried to get floor points from camera index %zu, while the index goes to 3", camIndex);
		exit(EXIT_FAILURE);
	}

	// acquire mutex used to protect the floorPointList and floorPointListValid
	std::unique_lock < std::mutex > myLock(floorPointListExportMutex);
	vector<linePointSt> retVal;
	if (floorPointListValid[camIndex]) {
		// copy data to new vector, so the mutex can be released and vector can be changed by the receiver thread
		retVal = floorPointList[camIndex];
		floorPointListValid[camIndex] = false; // wait again in next iteration until new data is provided
	}
	myLock.unlock();

	return retVal;
}

// get the latest obstacle points, blocks until new obstacle points have been received
vector<linePointSt> cameraReceive::getObstaclePointsWait(size_t camIndex) {
#ifndef NOROS
	TRACE_FUNCTION("");
#endif
	// acquire mutex used to protect the obstaclePointList and obstaclePointListValid
	std::unique_lock < std::mutex > myLock(obstaclePointListExportMutex);
	// wait for data
	if (camIndex == 0) {
		obstacleDetCondVar.wait(myLock, [] {return obstaclePointListValid[0];});
	} else if (camIndex == 1) {
		obstacleDetCondVar.wait(myLock, [] {return obstaclePointListValid[1];});
	} else if (camIndex == 2) {
		obstacleDetCondVar.wait(myLock, [] {return obstaclePointListValid[2];});
	} else if (camIndex == 3) {
		obstacleDetCondVar.wait(myLock, [] {return obstaclePointListValid[3];});
	} else {
		printf("ERROR     : tried to get obstacle points from camera index %zu, while the index goes to 3", camIndex);
		exit(EXIT_FAILURE);
	}

	// now the new list is available and the mutex is locked
	obstaclePointListValid[camIndex] = false; // wait again in next iteration until new data is provided

	// copy data to new vector, so the mutex can be released and vector can be changed by the receiver thread
	vector<linePointSt> retVal = obstaclePointList[camIndex];
	myLock.unlock();
	return retVal;
}

Mat cameraReceive::getCameraFrame(size_t ii) {
	cameraFrameExportMutex.lock();
	Mat retVal = cameraFrame[ii];
	cameraFrameExportMutex.unlock();
	return retVal;
}

double cameraReceive::getRecvDeltaTime(size_t camIndex) {
	if (camIndex >= 4) {
		printf("ERROR     : tried to get receive delta time for camera index %zu, while the index goes to 3", camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	statisticsExportMutex.lock();
	double retVal = receiveTimeDelta[camIndex];
	statisticsExportMutex.unlock();
	return retVal;
}

uint32_t cameraReceive::getAnaFrameCounter(size_t camIndex) {
	if (camIndex >= 4) {
		printf(
				"ERROR     : tried to get raspi analyze frame counter value for camera index %zu, while the index goes to 3",
				camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else {
		return anaFrameCounter[camIndex];
	}
}

uint16_t cameraReceive::getAnaApplUptime(size_t camIndex) {
	if (camIndex >= 4) {
		printf(
				"ERROR     : tried to get raspi analyze application uptime for camera index %zu, while the index goes to 3",
				camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else {
		return anaApplUptime[camIndex];
	}
}

uint8_t cameraReceive::getCamValAverage(size_t camIndex) {
	if (camIndex >= 4) {
		printf(
				"ERROR     : tried to get raspi analyze camera value average for camera index %zu, while the index goes to 3",
				camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else {
		return camValAverage[camIndex];
	}
}
