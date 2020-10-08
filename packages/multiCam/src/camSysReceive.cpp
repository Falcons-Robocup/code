 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "camSysReceive.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <net/if.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

camSysReceive::camSysReceive() {
	// create a normal UDP socket
	if ((multReceiveFd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cannot create UDP socket for receiving from raspiSystem, message: %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port
	int reuse = 1;
	if (setsockopt(multReceiveFd, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse)) < 0) {
		printf(
				"ERROR     : cannot configure port for multiple UTP sockets for receiving from raspiSystem, message: %s\n",
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// fill in the local address structure
	struct sockaddr_in localAddr;
	memset((void *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET; // Internet
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(33333); // raspi system port
	// bind to the local address / port
	if (::bind(multReceiveFd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		printf("ERROR     : cannot bind to UDP socket for receiving from raspiSystem, message: %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset((void *) &mreq, 0, sizeof(mreq)); // set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	// first try IP robot configuration
	mreq.imr_interface.s_addr = inet_addr("10.0.0.1");
	if (setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
		// fallback
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		if (setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
			printf("ERROR     : cannot join the multicast group from receiving from raspiSystem, %s\n",
					strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
	}

	printf("INFO      : raspiSystem receive multicast group address %s port %u\n", inet_ntoa(mreq.imr_multiaddr),
			ntohs(localAddr.sin_port));

#ifdef NONO
	// resolve directory from which multiCam is running
	// (verify the correct workspace and be able to set relative path to other required files)
	directory = get_current_dir_name();

	// remove the alone directory in case the executable is running as stand alone test
	char *match;
	const char *sub = "/alone";
	int len = strlen(sub);
	while ((match = strstr(directory, sub))) {
		*match = '\0';
		strcat(directory, match + len);
	}

#else
	directory = (char*) malloc(128);
	sprintf(directory, "%s/packages/multiCam", getenv("FALCONS_CODE_PATH"));

#endif
	printf("INFO      : multiCam directory %s\n", directory);

	struct timeval tv;
	gettimeofday(&tv, NULL);

	camSysExportMutex.lock();
	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		rxPacketCntExpected[camIndex] = 0; // initialize at 0
		rxPacketCntFirstCheck[camIndex] = true;
		camSys[camIndex].md5sumPart0 = 0xdeadc0de;
		camSys[camIndex].md5sumPart1 = 0xdeadc0de;
		camSys[camIndex].md5sumPart2 = 0xdeadc0de;
		camSys[camIndex].md5sumPart3 = 0xdeadc0de;
		camSys[camIndex].updated = false;
		camSys[camIndex].rebootTime = 0;
		camSys[camIndex].configAcknowledgeTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
		copyBuildRaspiAge[camIndex] = 0xffffffff; // long time ago since raspi has been updated
	}
	camSysExportMutex.unlock();

	// this class is used by multiple applications, the md5sum check should be run by only one program (multiCam)
	md5sumCheckEnable = false;

	storeImages = false;
	imageGrabPath = "/dev/shm";
	dateCode[0] = 0;

	camImageExportMutex.unlock();
}

void camSysReceive::setImageGrabPath(std::string imageGrabPath) {
	this->imageGrabPath = imageGrabPath;
	storeImages = true;
	printf("INFO      : store received camera jpg images in directory %s\n", imageGrabPath.c_str());
}

void camSysReceive::getLocalMd5sum() {
	printf("INFO      : update local md5sum, used to compare with files on raspi camera's\n");

	char command[128] = { 0 };
	sprintf(command, "cd %s/../../tools/raspiSetup; ./md5sumAllFiles >/dev/null", directory);
	printf("SHELL     : %s\n", command);
	if (system(command) < 0) {
		printf("ERROR     : cannot run %s command, message: %s\n", command, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// collect the md5sum of all relevant files, which is used to verify if camera's use the same version
	char fileName[128] = { 0 };
	sprintf(fileName, "%s/../../tools/raspiSetup/md5sumAllFiles.log", directory);
	FILE *fp = fopen(fileName, "r");
	if (fp == NULL) {
		printf("ERROR     : cannot open %s, message: %s\n", fileName, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	int retVal = fscanf(fp, "%8x%8x%8x%8x", &md5sumPart0Local, &md5sumPart1Local, &md5sumPart2Local, &md5sumPart3Local);
	if (retVal < 0) {
		printf("ERROR     : not able to retrieve md5sum from file pointer, message: %s\n", strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	fclose(fp);
	printf("INFO      : md5sumAlFiles %08x%08x%08x%08x\n", md5sumPart0Local, md5sumPart1Local, md5sumPart1Local,
			md5sumPart3Local);

}
void camSysReceive::md5sumCheck(size_t camIndex, uint32_t md5sumPart0Camera, uint32_t md5sumPart1Camera,
		uint32_t md5sumPart2Camera, uint32_t md5sumPart3Camera) {
	if ((md5sumPart0Camera != md5sumPart0Local) || (md5sumPart1Camera != md5sumPart1Local)
			|| (md5sumPart2Camera != md5sumPart2Local) || (md5sumPart3Camera != md5sumPart3Local)) {
		printf("WARNING   : cam %zu md5sum is %08x%08x%08x%08x but local md5sum is %08x%08x%08x%08x\n", camIndex,
				md5sumPart0Camera, md5sumPart1Camera, md5sumPart2Camera, md5sumPart3Camera, md5sumPart0Local,
				md5sumPart1Local, md5sumPart2Local, md5sumPart3Local);
		// the raspi will send a number of old md5sums before the system process is killed
		// only start the update process when it was not performed for a while
		if (copyBuildRaspiAge[camIndex] > 20) {
			copyBuildRaspiAge[camIndex] = 0;
			char command[128] = { 0 };
			sprintf(command, "cd %s/../../tools/raspiSetup; ./copyBuildRaspiSingle cam%zu &", directory, camIndex);
			printf("SHELL     : %s\n", command);
			if (system(command) < 0) {
				printf("ERROR     : cannot run %s command, message: %s\n", command, strerror(errno));
				fflush(stdout);
				exit(EXIT_FAILURE);
			}
			// update the local md5sum in case a local file was changed after multiCam was started
			getLocalMd5sum();
		} else {
			if (copyBuildRaspiAge[camIndex] < 0xffffffff) {
				copyBuildRaspiAge[camIndex]++;
			}
		}
	}
}

// backwards compatible with old statistics packet
void camSysReceive::md5sumPart0Check(size_t camIndex, uint32_t md5sumPart0Camera) {
	if (md5sumPart0Camera != md5sumPart0Local) {
		printf("WARNING   : cam %zu md5sumPart0 is %08x but local md5sumPart0 is %08x\n", camIndex, md5sumPart0Camera,
				md5sumPart0Local);
		// the raspi will send a number of old md5sums before the system process is killed
		// only start the update process when it was not performed for a while
		if (copyBuildRaspiAge[camIndex] > 20) {
			copyBuildRaspiAge[camIndex] = 0;
			char command[128] = { 0 };
			sprintf(command, "cd %s/../../tools/raspiSetup; ./copyBuildRaspiSingle cam%zu &", directory, camIndex);
			printf("SHELL     : %s\n", command);
			if (system(command) < 0) {
				printf("ERROR     : cannot run %s command, message: %s\n", command, strerror(errno));
				fflush(stdout);
				exit(EXIT_FAILURE);
			}
			// update the local md5sum in case a local file was changed after multiCam was started
			getLocalMd5sum();
		} else {
			if (copyBuildRaspiAge[camIndex] < 0xffffffff) {
				copyBuildRaspiAge[camIndex]++;
			}
		}
	}
}

void camSysReceive::storeImage(size_t camIndex, size_t receivedAlready) {
	char fileName[128] = { 0 };
	sprintf(fileName, "%s/cam%zu_%s.jpg", imageGrabPath.c_str(), camIndex, dateCode);
	// printf("INFO      : cam %zu file name %s\n", camIndex, fileName);

// open the output file
	FILE *fd = NULL;
	if ((fd = fopen(fileName, "wb")) == NULL) {
		printf("ERROR     : cam %zu cannot open output file %s, message %s\n", camIndex, fileName, strerror(errno));
		exit(EXIT_FAILURE);
	}

// write the jpg file
	fwrite(&camImage[camIndex], 1, receivedAlready, fd);

// close the file handle
	fclose(fd);

// verify if the stored image image is valid
	cv::Mat retVal = imread(fileName, cv::IMREAD_COLOR);

// there still might be a change the file does not contain a valid image
	if ((retVal.cols == SEND_IMAGE_WIDTH) && (retVal.rows == SEND_IMAGE_HEIGHT)) {
		// likely a valid image

		// use a symlink as pointer to the latest valid image
		char symLinkName[128] = { 0 };
		sprintf(symLinkName, "%s/cam%zu.jpg", imageGrabPath.c_str(), camIndex);
		// printf("INFO      : cam %zu symlink name %s\n", camIndex, symLinkName);

		struct stat statBuf;
		camImageExportMutex.lock();
		if (stat(symLinkName, &statBuf) == 0) {
			// file exist
			if (unlink(symLinkName) < 0) {
				printf("ERROR     : cam %zu cannot remove symlink %s, message: %s\n", camIndex, symLinkName,
						strerror(errno));
				fflush(stdout);
				exit(EXIT_FAILURE);
			}
		}

		if (symlink(fileName, symLinkName) < 0) {
			printf("ERROR     : cam %zu cannot create symlink %s, message: %s\n", camIndex, symLinkName,
					strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
		camImageExportMutex.unlock();
	}
}

void camSysReceive::receive() {
	while (1) {
		// block until data received
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		ssize_t nBytes = recvfrom(multReceiveFd, &rxPacket, sizeof(camPacketT), 0, (struct sockaddr *) &fromAddr,
				&fromAddrlen);
		if (nBytes < 0) {
			printf("ERROR     : reading from raspiSystem socket, message: %s\n", strerror(errno));
			close(multReceiveFd);
			exit(EXIT_FAILURE);
		}

		bool valid = true;
		if (rxPacket.size != nBytes) {
			printf("ERROR     : received %zd bytes, but expected %u bytes from raspySystem\n", nBytes, rxPacket.size);
			valid = false;
		} else {
			// printf("INFO      : received %u bytes from sender %s:%u\n", rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//        ntohs(fromAddr.sin_port));
		}

		size_t camIndex = rxPacket.id >> 6; // highest 2 bits contain the camera location (camera id)

		// checkout if we received the packet counter synchronizer from the raspiSystem process
		if (valid && ((rxPacket.id & 0x3f) == 3) && (rxPacket.pl.u32[0] == 0xcafe1234)) {
			// printf("INFO      : cam %zu received packet counter synchronizer %3u\n", camIndex, rxPacket.cnt);
			rxPacketCntExpected[camIndex] = rxPacket.cnt + 1;
			// this packet is not for further use
			valid = false;
		} else if (valid && (rxPacket.cnt != rxPacketCntExpected[camIndex])) {
			if (rxPacketCntFirstCheck[camIndex]) {
				rxPacketCntFirstCheck[camIndex] = false;
				rxPacketCntExpected[camIndex] = rxPacket.cnt + 1; // set the expected for the next received camera packet
			} else {
				printf(
						"WARNING   : cam %zu received camera packet counter %3u from raspiSystem, but expected packet counter %3u\n",
						camIndex, rxPacket.cnt, rxPacketCntExpected[camIndex]);
				rxPacketCntExpected[camIndex] = rxPacket.cnt + 1;
				valid = false;
			}
		} else {
			rxPacketCntExpected[camIndex]++;
		}

		if (valid) {
			// bits[7:6] define camera location
			if ((rxPacket.id & 0x3f) == 0) { // md5sum
				camSysExportMutex.lock();
				camSys[camIndex].md5sumPart0 = rxPacket.pl.u32[0]; // used to verify if raspi is using the correct files
				camSys[camIndex].md5sumPart1 = rxPacket.pl.u32[1];
				camSys[camIndex].md5sumPart2 = rxPacket.pl.u32[2];
				camSys[camIndex].md5sumPart3 = rxPacket.pl.u32[3];
				camSysExportMutex.unlock();
				if (md5sumCheckEnable) {
					md5sumCheck(camIndex, camSys[camIndex].md5sumPart0, camSys[camIndex].md5sumPart1,
							camSys[camIndex].md5sumPart2, camSys[camIndex].md5sumPart3);
				}
#ifdef NONO
				printf("INFO      : cam %zu md5sum %08x%08x%08x%08x\n", camIndex, camSys[camIndex].md5sumPart0,
						camSys[camIndex].md5sumPart1, camSys[camIndex].md5sumPart2, camSys[camIndex].md5sumPart3);
#endif

			} else if ((rxPacket.id & 0x3f) == 1) { // backwards compatible statistics
				camSysExportMutex.lock();
				camSys[camIndex].updated = true;
				camSys[camIndex].cpuTemp = rxPacket.pl.u8[0];
				camSys[camIndex].gpuTemp = rxPacket.pl.u8[1];
				camSys[camIndex].cpuLoad = rxPacket.pl.u8[2]; // warning: fixed point, divide by 32.0 to get correct value
				camSys[camIndex].cpuStatus = rxPacket.pl.u8[3]; // vcgencmd get_throttled, with upper bits shifted down
				camSys[camIndex].sysApplUptime = rxPacket.pl.u16[2]; // system application uptime in seconds
				camSys[camIndex].cpuUptime = rxPacket.pl.u16[3]; // CPU uptime in seconds
				camSys[camIndex].md5sumPart0 = rxPacket.pl.u32[2]; // used to verify if raspi is using the correct files
				camSys[camIndex].cmosTemp = rxPacket.pl.s8[12]; // imx219 camera sensor temperature

				// 0: under-voltage
				// 1: arm frequency capped
				// 2: currently throttled
				// 3: soft temp limit (added in 2018)

				// WARNING: on sender side bits 16-19 have been shifted to bits 4-7
				// 4: under-voltage has occurred
				// 5: arm frequency capped has occurred
				// 6: throttling has occurred
				// 7: soft temp limit (added in 2018)
				camSys[camIndex].underVoltage = (((camSys[camIndex].cpuStatus >> 0) & 0x1) == 1);
				camSys[camIndex].frequencyCapped = (((camSys[camIndex].cpuStatus >> 1) & 0x1) == 1);
				camSys[camIndex].throttling = (((camSys[camIndex].cpuStatus >> 2) & 0x1) == 1);
				camSys[camIndex].softTempLimit = (((camSys[camIndex].cpuStatus >> 3) & 0x1) == 1);
				camSys[camIndex].underVoltageOccured = (((camSys[camIndex].cpuStatus >> 4) & 0x1) == 1);
				camSys[camIndex].frequencyCappedOccured = (((camSys[camIndex].cpuStatus >> 5) & 0x1) == 1);
				camSys[camIndex].throttlingOccured = (((camSys[camIndex].cpuStatus >> 6) & 0x1) == 1);
				camSys[camIndex].softTempLimitOccured = (((camSys[camIndex].cpuStatus >> 7) & 0x1) == 1);
				latestCamIndex = camIndex;

				uint32_t md5sumPart0Camera = camSys[camIndex].md5sumPart0;
				camSysExportMutex.unlock();
				if (md5sumCheckEnable) {
					md5sumPart0Check(camIndex, md5sumPart0Camera);
				}
#ifdef NONO

				hhMmSsSt applUp = secondsToHhMmSs(camSys[camIndex].sysApplUptime);
				hhMmSsSt cpuUp = secondsToHhMmSs(camSys[camIndex].cpuUptime);
				printf(
						"INFO      : cam %zu cpu %2u gpu %2u cmos %2d load %4.2f v_err %u f_err %u t_err %u md5sum %08x cpu %02u:%02u:%02u appl %02u:%02u:%02u\n",
						camIndex, camSys[camIndex].cpuTemp, camSys[camIndex].gpuTemp, camSys[camIndex].cmosTemp,
						camSys[camIndex].cpuLoad/32.0, camSys[camIndex].underVoltage, camSys[camIndex].frequencyCapped,
						camSys[camIndex].throttling, camSys[camIndex].md5sumPart0, cpuUp.hours, cpuUp.minutes,
						cpuUp.seconds, applUp.hours, applUp.minutes, applUp.seconds);
#endif

			} else if ((rxPacket.id & 0x3f) == 4) { // statistics (pretty much the same as above, but without md5sum
				camSysExportMutex.lock();
				camSys[camIndex].updated = true;
				camSys[camIndex].cpuTemp = rxPacket.pl.u8[0];
				camSys[camIndex].gpuTemp = rxPacket.pl.u8[1];
				camSys[camIndex].cpuLoad = rxPacket.pl.u8[2]; // warning: fixed point, divide by 32.0 to get correct value
				camSys[camIndex].cpuStatus = rxPacket.pl.u8[3]; // vcgencmd get_throttled, with upper bits shifted down
				camSys[camIndex].sysApplUptime = rxPacket.pl.u16[2]; // system application uptime in seconds
				camSys[camIndex].cpuUptime = rxPacket.pl.u16[3]; // CPU uptime in seconds
				camSys[camIndex].cmosTemp = rxPacket.pl.s8[12]; // imx219 camera sensor temperature

				// 0: under-voltage
				// 1: arm frequency capped
				// 2: currently throttled

				// WARNING: on sender side bits 16-19 have been shifted to bits 4-7
				// 4: under-voltage has occurred
				// 5: arm frequency capped has occurred
				// 6: throttling has occurred
				camSys[camIndex].underVoltage = (((camSys[camIndex].cpuStatus >> 0) & 0x1) == 1);
				camSys[camIndex].frequencyCapped = (((camSys[camIndex].cpuStatus >> 1) & 0x1) == 1);
				camSys[camIndex].throttling = (((camSys[camIndex].cpuStatus >> 2) & 0x1) == 1);
				camSys[camIndex].underVoltageOccured = (((camSys[camIndex].cpuStatus >> 4) & 0x1) == 1);
				camSys[camIndex].frequencyCappedOccured = (((camSys[camIndex].cpuStatus >> 5) & 0x1) == 1);
				camSys[camIndex].throttlingOccured = (((camSys[camIndex].cpuStatus >> 6) & 0x1) == 1);
				latestCamIndex = camIndex;
				camSysExportMutex.unlock();

#ifdef NONO

				hhMmSsSt applUp = secondsToHhMmSs(camSys[camIndex].sysApplUptime);
				hhMmSsSt cpuUp = secondsToHhMmSs(camSys[camIndex].cpuUptime);
				printf(
						"INFO      : cam %zu cpu %2u gpu %2u cmos %2d load %4.2f v_err %u f_err %u t_err %u cpu %02u:%02u:%02u appl %02u:%02u:%02u\n",
						camIndex, camSys[camIndex].cpuTemp, camSys[camIndex].gpuTemp, camSys[camIndex].cmosTemp,
						camSys[camIndex].cpuLoad/32.0, camSys[camIndex].underVoltage, camSys[camIndex].frequencyCapped,
						camSys[camIndex].throttling, cpuUp.hours, cpuUp.minutes,
						cpuUp.seconds, applUp.hours, applUp.minutes, applUp.seconds);
#endif

			} else if ((rxPacket.id & 0x3f) == 6) { // compressed image
				if (storeImages) {
					// the image is send as multiple packets where each packet can hold up to 1466 bytes for the image data
					// for an image of 100kB this results in 69 packets
					// the first 4 + 2 bytes of the payload are used to indicate the frame counter and image part

					uint32_t frameCounter = rxPacket.pl.u32[0];
					uint16_t imagePart = rxPacket.pl.u16[2];
					if (imagePart == 0) {
						// printf("INFO      : cam %zu frameCounter %u\n", camIndex, frameCounter);
						for (size_t ii = 0; ii < 256; ii++) {
							imagePartValid[camIndex][ii] = false;
						}
					}
					imagePartValid[camIndex][imagePart] = true;

					// to related the 4 camera images, only update the file name (for all files)
					// a bit (10 * 10ms = 100ms) after the first packet has been received of camera 0
					// when the file name is directly changed after reception of the first packet of
					// camera 0, then it is possible the file of another camera still needs to be written
					if ((camIndex == 0) && (imagePart == 10)) {
						// use the same date and time for each camera

						struct timeval tv;
						time_t nowtime;
						struct tm *nowtm;

						gettimeofday(&tv, NULL);
						nowtime = tv.tv_sec;
						nowtm = localtime(&nowtime);
						// use current date and time as record file name
						strftime(dateCode, sizeof(dateCode), "%Y%m%d_%H%M%S", nowtm);
					}

					size_t bufferIndex = imagePart * (1468 - 4 - 2); // TODO: update and use CAM_PACKET_PAYLOAD_SIZE again
					size_t payloadSize = rxPacket.size - CAM_PACKET_HEADER_SIZE - 4 - 2; // 4 + 2 bytes used for the frame counter and imagePart

					size_t receivedAlready = bufferIndex + payloadSize;
					if (receivedAlready > sizeof(camImage[camIndex])) {
						printf(
								"ERROR     : cam %zu total received %zu bytes which do not fit in cam receive buffer of %zu bytes\n",
								camIndex, receivedAlready, sizeof(camImage[camIndex]));
						fflush(stdout);
						// TODO enable again, when all robot's make use of the packet including frame counter exit(EXIT_FAILURE);
					} else {
						// start the copy from byte 6 (byte 0 to 3 used for frame counter, byte 4 and 5 for imagePart)
						memcpy(&camImage[camIndex][bufferIndex], &rxPacket.pl.u8[6], payloadSize);
						// only the last part will have a different size
						if (payloadSize != (1468 - 4 - 2)) {
							// last image part
							// printf("INFO      : cam %zu compressed file size %zu\n", camIndex, receivedAlready);

							// determine if all packets have been received, otherwise do not store the image
							size_t missedPackets = 0;
							for (size_t ii = 0; ii < (size_t) imagePart; ii++) {
								if ((imagePart < 256) && (imagePartValid[camIndex][ii] == false)) {
									missedPackets++;
								}

							}
							if (missedPackets == 0) {
								storeImage(camIndex, receivedAlready);

							} else {
								printf(
										"WARNING   : cam %zu cannot write compressed image frame counter %u because %zu of %u packets not received\n",
										camIndex, frameCounter, missedPackets, imagePart);

							}

						} // if (payloadSize != (1468 - 4 - 2)) {
					} // if (receivedAlready > sizeof(camImage[camIndex])
				} // if ( storeImage )
				  // printf("INFO      : cam %zu payload size %zu\n", camIndex, payloadSize);
			} else if ((rxPacket.id & 0x3f) == 2) { // reboot request
				struct timeval tv;
				gettimeofday(&tv, NULL);
				camSysExportMutex.lock();
				camSys[camIndex].rebootTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
				latestCamIndex = camIndex;
				camSysExportMutex.unlock();

				printf("INFO      : cam %zu #### is going to reboot ####\n", camIndex);
				fflush(stdout);

			} else if ((rxPacket.id & 0x3f) == 7) { // raspi returned config acknowledge packet, to measure round trip UDP latency
				struct timeval tv;
				gettimeofday(&tv, NULL);
				camSysExportMutex.lock();
				camSys[camIndex].configAcknowledgeTime = 1.0 * tv.tv_sec + 0.000001 * tv.tv_usec;
				latestCamIndex = camIndex;
				camSysExportMutex.unlock();

				// printf("INFO      : cam %zu received config acknowledge\n", camIndex);
			} else {
				printf("ERROR     : cam %zu unknown packet id %u from raspiSystem\n", camIndex, rxPacket.id);
				fflush(stdout);
			}
		} // if (valid
	} // while ( 1
} // void camSysReceive::receive()

camSystemSt camSysReceive::getCamSystem(size_t camIndex) {
	camSystemSt retVal;
	if (camIndex >= 4) {
		printf("ERROR     : requested invalid camIndex %zu when requesting raspiSystem information\n", camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else {
		camSysExportMutex.lock();
		retVal = camSys[camIndex];
		camSys[camIndex].updated = false;
		camSysExportMutex.unlock();
	}
	return retVal;
}

cv::Mat camSysReceive::getCameraFrame(size_t camIndex) {
	cv::Mat retVal;
	bool valid = false;

	if (camIndex >= 4) {
		printf("ERROR     : requested invalid camIndex %zu when requesting raspiSystem information\n", camIndex);
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (storeImages) {
		// the symlink points to the latest valid image (if any)
		char symLinkName[128] = { 0 };
		sprintf(symLinkName, "%s/cam%zu.jpg", imageGrabPath.c_str(), camIndex);
		// printf("INFO      : cam %zu symlink name %s\n", camIndex, symLinkName);
		camImageExportMutex.lock();
		retVal = imread(symLinkName, cv::IMREAD_COLOR);
		camImageExportMutex.unlock();

		// there still might be a change the file does not contain a valid image
		if ((retVal.cols == SEND_IMAGE_WIDTH) && (retVal.rows == SEND_IMAGE_HEIGHT)) {
			valid = true;
		}
	}

	if (valid) {
		return retVal;
	} else {
		return Mat::zeros(SEND_IMAGE_HEIGHT, SEND_IMAGE_WIDTH, CV_8UC3);
	}
}

hhMmSsSt camSysReceive::secondsToHhMmSs(uint16_t input) {
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
