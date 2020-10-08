// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// raspi system controls:
//  - Sony imx219 camera (i2c)
//  - reboot
//  - power cycle
//  - update the boot-up scripts
//  - update grabber source code and re-compile
//  - update analyzer source code and re-compile
//  - update system code and re-compile

// raspi system provides the following information to the CPU box:
//  - CPU temperature
//  - GPU temperature
//  - camera temperature (i2c)
//  - power under voltage detection
//  - system uptime
//  - grabber, analyzer and system source code version

// range for valid packed id's for raspi system are 64 to 80

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h> // for O_RDWR
#include <getopt.h>
#include <inttypes.h>
#if defined(__arm__) || defined(__aarch64__)
#include <linux/i2c.h>
#endif
#include <linux/i2c-dev.h>
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
#include <sys/ioctl.h> // for i2c ioctl
#include <sys/mman.h> // for shared memory
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h> // usleep

#include "raspiDefaults.hpp"

// proto type from raspiRgbToBmp
void rgbToBmp(size_t camIndex, char *inputFileName, char *outputFileName, bool reduce, bool verbose);

bool keepRunning = true;
char grabFileName[256] = "/dev/shm/raspiGrab";
uint32_t frameCounterPrev = 0;

uint8_t camIndex = 0; // provided through command line
int i2cFd = 0;
int multReceiveFd = 0;
int multReceiveLoopBackFd = 0;
int multSendFd = 0;
struct sockaddr_in toAddr;
uint8_t cpuStatus = 0;
uint8_t gpuTemp = 0;
uint8_t cpuTemp = 0;
uint8_t cpuLoad = 0;
uint16_t systemApplicationUptime = 0;
uint16_t cpuUptime = 0;
bool cmosTempEnabled = false;
int8_t cmosTemp = 0;
pid_t processId = 0;
bool valgrind = false; // used for x86_64 grabber uptime testing

pthread_t sendFrameThreadId;
pthread_t raspiConfigThreadId;
pthread_mutex_t sendMutex;
pthread_mutex_t i2cMutex;

bool statisticsDone = false;

uint32_t md5sumPart0 = 0;
uint32_t md5sumPart1 = 0;
uint32_t md5sumPart2 = 0;
uint32_t md5sumPart3 = 0;

// camera clocks (imx219)
uint16_t EXCK_FREQ = EXCK_FREQ_DEFAULT; // 0x012a-0x012b
uint16_t VTPXCK_DIV = VTPXCK_DIV_DEFAULT; // 0x0301
uint16_t VTSYCK_DIV = VTSYCK_DIV_DEFAULT; // 0x0303
uint16_t PREPLLCK_VT_DIV = PREPLLCK_VT_DIV_DEFAULT; // 0x0304
uint16_t PREPLLCK_OP_DIV = PREPLLCK_OP_DIV_DEFAULT; // 0x0305
uint16_t PLL_VT_MPY = PLL_VT_MPY_DEFAULT; // 0x0306-0x0307
uint16_t OPPXCK_DIV = OPPXCK_DIV_DEFAULT; // 0x0309
uint16_t OPSYCK_DIV = OPSYCK_DIV_DEFAULT; // 0x030b
uint16_t PLL_OP_MPY = PLL_OP_MPY_DEFAULT; // 0x030c-0x030d

uint16_t EXCK_FREQ_PREV = 0xffff; // 0x012a-0x012b
uint16_t VTPXCK_DIV_PREV = 0xffff; // 0x0301
uint16_t VTSYCK_DIV_PREV = 0xffff; // 0x0303
uint16_t PREPLLCK_VT_DIV_PREV = 0xffff; // 0x0304
uint16_t PREPLLCK_OP_DIV_PREV = 0xffff; // 0x0305
uint16_t PLL_VT_MPY_PREV = 0xffff; // 0x0306-0x0307
uint16_t OPPXCK_DIV_PREV = 0xffff; // 0x0309
uint16_t OPSYCK_DIV_PREV = 0xffff; // 0x030b
uint16_t PLL_OP_MPY_PREV = 0xffff; // 0x030c-0x030d

// camera control and white balance (software)
uint16_t cameraReset = CAMERA_RESET;
uint16_t cameraVerbose = CAMERA_VERBOSE;
uint16_t testPattern = TEST_PATTERN;
uint16_t analogGain = ANALOG_GAIN;
uint16_t shutter = SHUTTER;
uint16_t blackLevel = BLACK_LEVEL;

uint16_t cameraResetPrev = 0xffff;
uint16_t cameraVerbosePrev = 0xffff;
uint16_t testPatternPrev = 0xffff;
uint16_t analogGainPrev = 0xffff;
uint16_t shutterPrev = 0xffff;
uint16_t blackLevelPrev = 0xffff;

// camera dimensions
uint16_t lines = LINES; // 0x0160-0x0161
uint16_t pixelsBase = PIXELS; // 0x0162-0x0163
int8_t pixelsOffset = 0;
uint16_t xStart = X_START; // 0x0164-0x0165
uint16_t xEnd = X_END; // 0x0166-0x0167
uint16_t xSize = X_SIZE; // 0x016c-0x016d
uint16_t yStart = Y_START; // 0x0168-0x0169
uint16_t yEnd = Y_END; // 0x016a-0x016b
uint16_t ySize = Y_SIZE; // 0x016e-0x016f
uint16_t imageOrientation = IMAGE_ORIENTATION; // 0x0172

uint16_t linesPrev = 0xffff; // 0x0160-0x0161
uint16_t pixelsPrev = 0xffff; // 0x0162-0x0163
uint16_t xStartPrev = 0xffff; // 0x0164-0x0165
uint16_t xEndPrev = 0xffff; // 0x0166-0x0167
uint16_t xSizePrev = 0xffff; // 0x016c-0x016d
uint16_t yStartPrev = 0xffff; // 0x0168-0x0169
uint16_t yEndPrev = 0xffff; // 0x016a-0x016b
uint16_t ySizePrev = 0xffff; // 0x016e-0x016f
uint16_t imageOrientationPrev = 0xffff; // 0x0172

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
// #define IPV4_MAX_SIZE 65535 // 2^16 - 1, TODO: why not 65536 ?
#define MTU 1500
#define IPV4_IP_HEADER 20
#define UDP_PACKET_MAX_SIZE (MTU - IPV4_IP_HEADER) // 1480
#define UDP_HEADER_SIZE 8
#define UDP_PACKET_PAYLOAD_SIZE (UDP_PACKET_MAX_SIZE - UDP_HEADER_SIZE) // 1472
#define CAM_PACKET_PAYLOAD_SIZE (UDP_PACKET_PAYLOAD_SIZE - CAM_PACKET_HEADER_SIZE) // 1468

packetT txPacket;

void multiCastSendSetup() {
	// create a normal UDP socket
	if ((multSendFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cam %u system cannot create UDP socket for sending!, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address
	memset((char *) &toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(33333); // data port for raspi system

	printf("INFO      : cam %u system mulitcast send    IP group address %s port %u\n", camIndex,
			inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));

	pthread_mutex_lock(&sendMutex);
	txPacket.cnt = 0; // the value incremented before use, so the first send packet counter will be 1 (instead of 0)
	pthread_mutex_unlock(&sendMutex);

}

void multiCastSendClose() {
	close(multSendFd);
}

// is called within the sendMutex lock
inline void sendThePacket(char *printString) {
	if (txPacket.size > UDP_PACKET_PAYLOAD_SIZE) {
		printf("ERROR     : cam %u system UDP payload of %u bytes exceeds MTU %u bytes for %s packet\n", camIndex,
				txPacket.size, UDP_PACKET_PAYLOAD_SIZE, printString);
		fflush(stdout);
	} else {
		ssize_t sendAmount = sendto(multSendFd, &txPacket, txPacket.size, 0, (struct sockaddr *) &toAddr,
				sizeof(toAddr));
		if (sendAmount < 0) {
			printf("ERROR     : cam %u system cannot send %s packet, message %s\n", camIndex, printString,
					strerror(errno));
			printf("WARNING   : cam %u system exit now\n", camIndex);
			fflush(stdout);
			exit(EXIT_FAILURE);
		} else if (sendAmount != txPacket.size) {
			printf("WARNING   : cam %u system only %zd bytes instead of %u for %s packet\n", camIndex, sendAmount,
					txPacket.size, printString);
			fflush(stdout);
		}
	}
}

// send first packet to synchronize packet counter with CPU
inline void packetCntSyncSend() {
	pthread_mutex_lock(&sendMutex);
	txPacket.pl.u32[0] = 0xcafe1234; // magic payload
	txPacket.id = camIndex * 64 + 3;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 4;

	sendThePacket((char *) "packet count sync");
	pthread_mutex_unlock(&sendMutex);
}

// send md5sum of used files on raspi to x86_64, used to trigger the update process on the x86_64
inline void md5sumSend() {
	pthread_mutex_lock(&sendMutex);
	txPacket.pl.u32[0] = md5sumPart0; // used to check if correct files are used
	txPacket.pl.u32[1] = md5sumPart1;
	txPacket.pl.u32[2] = md5sumPart2;
	txPacket.pl.u32[3] = md5sumPart3;

	txPacket.id = camIndex * 64 + 0;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 4 * 4;

	sendThePacket((char *) "md5sum");
	pthread_mutex_unlock(&sendMutex);
}

// send the compressed image to x86_64
inline void imageSend(char *inputFileName, uint32_t frameCounter, bool verbose) {
	// determine input file size
	struct stat st;
	stat(inputFileName, &st);
	size_t fileSize = st.st_size;
	if (verbose) {
		printf("INFO      : cam %u system compressed image file size %.1fkB\n", camIndex, fileSize / 1000.0);
	}

	// open the file
	FILE *fd = NULL;
	if ((fd = fopen(inputFileName, "rb")) == NULL) {
		printf("ERROR     : cam %u system cannot read file %s, message %s\n", camIndex, inputFileName, strerror(errno));
		exit(EXIT_FAILURE);
	}

	// allocate the memory for the input image (input file)
	uint8_t *buffer;
	buffer = (uint8_t *) malloc(fileSize);
	if (buffer == NULL) {
		printf("ERROR     : cam %u system cannot allocate %zu bytes for input image, message %s\n", camIndex, fileSize,
				strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (verbose) {
		printf("INFO      : cam %u system first buffer pointer %p last buffer pointer %p size %zu bytes\n", camIndex,
				buffer, &buffer[fileSize - 1], &buffer[fileSize - 1] - buffer + 1);
	}

	// copy the file into the buffer
	size_t nRead = fread(buffer, 1, fileSize, fd);
	if (nRead != fileSize) {
		printf("ERROR     : cam %u system only %zu bytes of %zu have been read from the file\n", camIndex, nRead,
				fileSize);
		exit(EXIT_FAILURE);
	}
	fclose(fd);

	// send the buffer to the x86_64
	uint16_t imagePart = 0; // which part of the compressed image
	size_t offset = 0;
	// the receiver has a maximal buffer size of 800*608*3 = 1459200 (uncompressed) and 1459200/1466 ~= 995
	while ((offset < fileSize) && (imagePart < 995)) {
		pthread_mutex_lock(&sendMutex);
		txPacket.pl.u32[0] = frameCounter;
		txPacket.pl.u16[2] = imagePart;
		size_t remainder = fileSize - offset;
		if (remainder > ( CAM_PACKET_PAYLOAD_SIZE - 4 - 2)) { // 4 + 2 bytes for the frame counter and imagePart
			remainder = CAM_PACKET_PAYLOAD_SIZE - 4 - 2;
		}

		// start the copy from byte 6 (byte 0 to 3 used for frame counter, byte 4 and 5 for imagePart)
		memcpy((void *) &txPacket.pl.u8[6], (void *) &buffer[offset], remainder);
		offset += remainder;

		txPacket.id = camIndex * 64 + 6;
		txPacket.cnt++; // wrap around at 255
		txPacket.size = CAM_PACKET_HEADER_SIZE + 4 + 2 + remainder;

		imagePart++;

		sendThePacket((char *) "image");
		// compressed image is around 70kB/1466 ~= 48 packets
		// the system period should be around 1 second
		// 1000ms/48 ~= 20.8ms
		pthread_mutex_unlock(&sendMutex);

		usleep(10000); // 10ms, so there is enough head room to send the complete image in 1 second
	}
	free(buffer);
}

void multiCastReceiveSetup() {
	// create a normal UDP socket
	if ((multReceiveFd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf("ERROR     : cam %u system cannot create UDP socket for receiving!, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// allow multiple sockets to use the same port (required when multiple simulation instances are running on x86)
	int reuse = 1;
	if (setsockopt(multReceiveFd, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse)) < 0) {
		printf("ERROR     : cam %u system cannot configure port for multiple UDP sockets, message: %s\n", camIndex,
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
		printf("ERROR     : cam %u system cannot bind to UDP socket for receiving, %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// join the multicast group
	struct ip_mreq mreq;
	memset(&mreq, 0, sizeof(mreq)); // set all to zero
	mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
		printf("ERROR     : cam %u system cannot join the multicast group for receiving, %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	printf("INFO      : cam %u system multicast receive IP group address %s port %u\n", camIndex,
			inet_ntoa(mreq.imr_multiaddr), ntohs(localAddr.sin_port));
}

void multiCastReceiveClose() {
	close(multReceiveFd);
}

// multiCastReceiveLoopBackSetup used to provide data from this application the receive configuration port of this same application (loop back)
void multiCastReceiveLoopBackSetup() {
	// create a normal UDP socket
	if ((multReceiveLoopBackFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		printf(
				"ERROR     : cam %u system cannot create UDP socket for sending to local configuration port!, message %s\n",
				camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	// setup the multicast destination address for control
	memset((char *) &toAddr, 0, sizeof(toAddr));
	toAddr.sin_family = AF_INET; // Internet
	toAddr.sin_addr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
	toAddr.sin_port = htons(22222); // configuration port

	printf("INFO      : cam %u system mulitcast loop back IP group address %s port %u\n", camIndex,
			inet_ntoa(toAddr.sin_addr), ntohs(toAddr.sin_port));
}

void multiCastReceiveLoopBackClose() {
	close(multReceiveLoopBackFd);
}

void multiCastReceiveLoopBackStop() {
	packetT lbTxPacket;
	lbTxPacket.pl.u32[0] = 0xdead0099; // magic for stop
	lbTxPacket.id = 68;
	lbTxPacket.cnt = 0; // packet counter is not used on receive because data can come from unrelated sources
	lbTxPacket.size = CAM_PACKET_HEADER_SIZE + 4;

	// sendBuf.size is the number of bytes to send
	ssize_t sendAmount = sendto(multReceiveLoopBackFd, &lbTxPacket, lbTxPacket.size, 0, (struct sockaddr *) &toAddr,
			sizeof(toAddr));
	if (sendAmount < 0) {
		printf("ERROR     : cam %u system cannot send application exit loop back message, %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	} else if (sendAmount != lbTxPacket.size) {
		printf("WARNING   : cam %u system only send %zd bytes to loop back instead of %d bytes\n", camIndex, sendAmount,
				lbTxPacket.size);
	}
}

// send old type statistics packet, that will work with old software and if needed trigger the update
inline void backWardsCompatibleStaticsSends() {
	pthread_mutex_lock(&sendMutex);
	txPacket.pl.u8[0] = cpuTemp;
	txPacket.pl.u8[1] = gpuTemp;
	txPacket.pl.u8[2] = cpuLoad;
	txPacket.pl.u8[3] = cpuStatus; // including under voltage detection, bits 16-19 have moved to 4-7
	txPacket.pl.u16[2] = systemApplicationUptime; // application uptime in seconds (wrap around at 18.2 hours)
	txPacket.pl.u16[3] = cpuUptime; // CPU uptime in seconds (wrap around at 18.2 hours)
	txPacket.pl.u32[2] = md5sumPart0; // used to check if correct files are used
	txPacket.pl.s8[12] = cmosTemp; // imx219 camera sensor temperature

	txPacket.id = camIndex * 64 + 1;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 13;

	sendThePacket((char *) "statistics");
	pthread_mutex_unlock(&sendMutex);
}

// send statistics
inline void statisticsSend() {
	pthread_mutex_lock(&sendMutex);
	txPacket.pl.u8[0] = cpuTemp;
	txPacket.pl.u8[1] = gpuTemp;
	txPacket.pl.u8[2] = cpuLoad;
	txPacket.pl.u8[3] = cpuStatus; // including under voltage detection, bits 16-19 have moved to 4-7
	txPacket.pl.u16[2] = systemApplicationUptime; // application uptime in seconds (wrap around at 18.2 hours)
	txPacket.pl.u16[3] = cpuUptime; // CPU uptime in seconds (wrap around at 18.2 hours)
	txPacket.pl.s8[12] = cmosTemp; // imx219 camera sensor temperature

	txPacket.id = camIndex * 64 + 4;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE + 9;

	sendThePacket((char *) "statistics");
	pthread_mutex_unlock(&sendMutex);
}

static inline void sleepKillReboot(int time) {
	printf("INFO      : cam %u system inform CPU the raspi is going to reboot\n", camIndex);
	//inform CPU the raspi is starting to reboot
	pthread_mutex_lock(&sendMutex);
	txPacket.id = camIndex * 64 + 2;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE;

	sendThePacket((char *) "sleep kill reboot");
	pthread_mutex_lock(&sendMutex);
	printf("INFO      : cam %u system reboot in %d seconds\n", camIndex, time);
	char command[64];
	sprintf(command, "./sleepKillReboot %d &", time);

	if (system(command) < 0) {
		printf("ERROR     : cam %u system shell cannot execute %s, message: %s\n", camIndex, command, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
}

// send the config acknowledge packet, used to measure the round trip UDP latency
inline void sendConfigAcknowledge() {
	pthread_mutex_lock(&sendMutex);

	txPacket.id = camIndex * 64 + 7;
	txPacket.cnt++; // wrap around at 255
	txPacket.size = CAM_PACKET_HEADER_SIZE;

	sendThePacket((char *) "config acknowledge");
	pthread_mutex_unlock(&sendMutex);
}

#if defined(__arm__) || defined(__aarch64__)

#define IMX219_I2C_ADDRESS 0x10
// the raspi camera security chip (Atmel) is on address 0x64
// checkout with: i2cdetect -y 0

static inline uint8_t i2cRegRead(const uint16_t regAddr) {
	uint8_t inbuf[2];
	uint8_t outbuf[2];
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];

	/*
	 * In order to read a register, we first do a "dummy write" by writing
	 * 0 bytes to the register we want to read from.  This is similar to
	 * the packet in set_i2c_register, except it's 1 byte rather than 2.
	 */
	outbuf[0] = regAddr >> 8;
	outbuf[1] = regAddr & 0xFF;
	messages[0].addr = IMX219_I2C_ADDRESS;
	messages[0].flags = 0;
	messages[0].len = 2,			//sizeof(outbuf);
	messages[0].buf = outbuf;
	/* The data will get returned in this structure */
	messages[1].addr = IMX219_I2C_ADDRESS;
	messages[1].flags = I2C_M_RD; /* | I2C_M_NOSTART*/
	messages[1].len = 1,			//sizeof(inbuf);
	messages[1].buf = inbuf;

	/* Send the request to the kernel and get the result back */
	packets.msgs = messages;
	packets.nmsgs = 2;
	pthread_mutex_lock(&i2cMutex);
	if (ioctl(i2cFd, I2C_RDWR, &packets) < 0) {
		printf("ERROR     : cam %u system unable to read i2c, message: %s\n", camIndex, strerror(errno));
		sleepKillReboot(10);
	}
	pthread_mutex_unlock(&i2cMutex);
	return inbuf[0];
}

static inline void i2cRegWrite(const uint16_t regAddr, const uint8_t regData) {
	unsigned char outbuf[3];
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[1];

	messages[0].addr = IMX219_I2C_ADDRESS;
	messages[0].flags = 0;
	messages[0].len = sizeof(outbuf);
	messages[0].buf = outbuf;
	/* The first byte indicates which register we'll write */
	outbuf[0] = regAddr >> 8;
	outbuf[1] = regAddr & 0xFF;
	outbuf[2] = regData;
	/* Transfer the i2c packets to the kernel and verify it worked */
	packets.msgs = messages;
	packets.nmsgs = 1;
	pthread_mutex_lock(&i2cMutex);
	if (ioctl(i2cFd, I2C_RDWR, &packets) < 0) {
		printf("ERROR     : cam %u system unable to write i2c, message: %s\n", camIndex, strerror(errno));
		sleepKillReboot(10);
	}
	pthread_mutex_unlock(&i2cMutex);
// unsigned char readData = i2cRegRead( i2cFd, regAddr );
// printf("INFO    : cam %u system i2c write 0x%02x read 0x%02x\n", camIndex, regData, readData);
}

#else
inline uint8_t i2cRegRead(const uint16_t regAddr) {
	(void) regAddr;
	// empty function to catch the i2c read on the x86
	return 0;
}

inline void i2cRegWrite(const uint16_t regAddr, const uint8_t regData) {
	(void) regAddr;
	(void) regData;
	// empty function to catch the i2c read on the x86
}
#endif

static inline void raspiCameraGetIdentifiers() {
	// get the camera model, lot id, waver number and chip number (likely from camera NVM)
	uint16_t addr = 0;
	uint32_t model = i2cRegRead(addr) << 8;
	addr = 1;
	model |= i2cRegRead(addr);
	addr = 4;
	uint32_t lotId = i2cRegRead(addr) << 16;
	addr = 5;
	lotId |= i2cRegRead(addr) << 8;
	addr = 6;
	lotId |= i2cRegRead(addr);
	addr = 7;
	uint32_t waferNum = i2cRegRead(addr);
	addr = 0x0d;
	uint32_t chipNum = i2cRegRead(addr) << 8;
	addr = 0x0e;
	chipNum |= i2cRegRead(addr);
	printf("INFO      : cam %u system model 0x%04x lot ID 0x%06x, wafer 0x%02x chip 0x%04x\n", camIndex, model, lotId,
			waferNum, chipNum);
	fflush(stdout);
}

static inline void raspiCameraConfigDump() {
	// get the camera model, lot id, waver number and chip number (likely from camera NVM)
	uint16_t addr = 0;
	uint32_t model = i2cRegRead(addr) << 8;
	addr = 1;
	model |= i2cRegRead(addr);
	addr = 4;
	uint32_t lotId = i2cRegRead(addr) << 16;
	addr = 5;
	lotId |= i2cRegRead(addr) << 8;
	addr = 6;
	lotId |= i2cRegRead(addr);
	addr = 7;
	uint32_t waferNum = i2cRegRead(addr);
	addr = 0x0d;
	uint32_t chipNum = i2cRegRead(addr) << 8;
	addr = 0x0e;
	chipNum |= i2cRegRead(addr);
	printf("INFO      : cam %u system model 0x%04x lot ID 0x%06x, wafer 0x%02x chip 0x%04x\n", camIndex, model, lotId,
			waferNum, chipNum);

	uint8_t readback;

	for (addr = 0x0000; addr <= 0x001b; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	for (addr = 0x0040; addr <= 0x0047; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	for (addr = 0x0080; addr <= 0x0093; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	for (addr = 0x00c0; addr <= 0x00c5; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	for (addr = 0x0100; addr <= 0x018d; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	for (addr = 0x0300; addr <= 0x030d; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	for (addr = 0x1000; addr <= 0x11c7; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
		if (addr % 0x10 == 0x0f) {
			printf("\n");
		}
	}
	if (addr % 0x10 != 0x0f) {
		printf("\n");
	}

	// TODO: investigate why the following registers are printed and what they mean
	for (addr = 0xd1e0; addr <= 0xd1eb; addr++) {
		readback = i2cRegRead(addr);
		if (addr % 0x10 == 0) {
			printf("0x%04x :", addr);
		}
		printf(" 0x%02x", readback);
	}
	printf("\n");
	fflush(stdout);
}

static inline void raspiCameraConfigUpdate() {
	if (EXCK_FREQ != EXCK_FREQ_PREV) {
		printf("INFO      : cam %d system 0x012a : 0x%04x (%4u) set EXCK_FREQ (to 24MHz)\n", camIndex, EXCK_FREQ,
				EXCK_FREQ);
		fflush(stdout);
		i2cRegWrite(0x012a, (uint8_t) (EXCK_FREQ >> 8)); // total 16 bits
		i2cRegWrite(0x012b, (uint8_t) (EXCK_FREQ & 0xff));
		EXCK_FREQ_PREV = EXCK_FREQ;
	}
	if (VTPXCK_DIV != VTPXCK_DIV_PREV) {
		printf("INFO      : cam %d system 0x0301 :   0x%02x (%4u) set VTPXCK_DIV (video timing pixel clock divider)\n",
				camIndex, VTPXCK_DIV, VTPXCK_DIV);
		i2cRegWrite(0x0301, (uint8_t) (VTPXCK_DIV & 0xff)); // 5 bits
		VTPXCK_DIV_PREV = VTPXCK_DIV;

		if ((VTPXCK_DIV == 0x4) || (VTPXCK_DIV == 0x8)) {
			i2cRegWrite(0x0155, 1); // enable compression 10 to 8 bits compression
			printf("INFO      : cam %d system 0x0155 :   0x01 (   1 ) enable 10 to 8 bits compression)\n", camIndex);
		} else {
			i2cRegWrite(0x0155, 0); // disable compression 10 to 8 bits compression
			printf("INFO      : cam %d system 0x0155 :   0x00 (   0 ) disable 10 to 8 bits compression)\n", camIndex);
		}

		fflush(stdout);

	}
	if (VTSYCK_DIV != VTSYCK_DIV_PREV) {
		printf("INFO      : cam %d system 0x0303 :   0x%02x (%4u) set VTSYCK_DIV (video timing system clock divider)\n",
				camIndex, VTSYCK_DIV, VTSYCK_DIV);
		fflush(stdout);
		i2cRegWrite(0x0303, (uint8_t) (VTSYCK_DIV & 0xff)); // 2 bits
		VTSYCK_DIV_PREV = VTSYCK_DIV;
	}
	if (PREPLLCK_VT_DIV != PREPLLCK_VT_DIV_PREV) {
		printf(
				"INFO      : cam %d system 0x0304 :   0x%02x (%4u) set PREPLLCK_VT_DIV (pre PLL clock video timing system divider\n",
				camIndex, PREPLLCK_VT_DIV, PREPLLCK_VT_DIV);
		fflush(stdout);
		i2cRegWrite(0x0304, (uint8_t) (PREPLLCK_VT_DIV & 0xff)); // 8 bits
		PREPLLCK_VT_DIV_PREV = PREPLLCK_VT_DIV;
	}
	if (PREPLLCK_OP_DIV != PREPLLCK_OP_DIV_PREV) {
		printf(
				"INFO      : cam %d system 0x0305 :   0x%02x (%4u) set PREPLLCK_OP_DIV (pre PLL clock output system divider)\n",
				camIndex, PREPLLCK_OP_DIV, PREPLLCK_OP_DIV);
		fflush(stdout);
		i2cRegWrite(0x0305, (uint8_t) (PREPLLCK_OP_DIV & 0xff)); // 8 bits
		PREPLLCK_OP_DIV_PREV = PREPLLCK_OP_DIV;
	}
	if (PLL_VT_MPY != PLL_VT_MPY_PREV) {
		i2cRegWrite(0x306, (uint8_t) (PLL_VT_MPY >> 8)); // total 11 bits
		i2cRegWrite(0x307, (uint8_t) (PLL_VT_MPY & 0xff));
		PLL_VT_MPY_PREV = PLL_VT_MPY;

		// read back the just written value
		uint32_t value = i2cRegRead(0x306) << 8;
		value |= i2cRegRead(0x307);
		if (value != PLL_VT_MPY) {
			printf(
					"ERROR     : cam %d system 0x0306 : 0x%04x (%4u) set PLL_VT_MPY (PLL video timing system multiplier, but should be 0x%04x)\n",
					camIndex, value, value, PLL_VT_MPY);
		} else {
			printf(
					"INFO      : cam %d system 0x0306 : 0x%04x (%4u) set PLL_VT_MPY (PLL video timing system multiplier)\n",
					camIndex, value, value);
		}
		fflush(stdout);
	}
	if (OPPXCK_DIV != OPPXCK_DIV_PREV) {
		printf("INFO      : cam %d system 0x0309 :   0x%02x (%4u) set OPPXCK_DIV (output pixel clock divider)\n",
				camIndex, OPPXCK_DIV, OPPXCK_DIV);
		fflush(stdout);
		i2cRegWrite(0x0309, (uint8_t) (OPPXCK_DIV & 0xff)); // 5 bits
		OPPXCK_DIV_PREV = OPPXCK_DIV;
	}
	if (OPSYCK_DIV != OPSYCK_DIV_PREV) {
		printf("INFO      : cam %d system 0x030b :   0x%02x (%4u) set OPSYCK_DIV (output system clock divider)\n",
				camIndex, OPSYCK_DIV, OPSYCK_DIV);
		fflush(stdout);
		i2cRegWrite(0x030b, (uint8_t) (OPSYCK_DIV & 0xff)); // 2 bits
		OPSYCK_DIV_PREV = OPSYCK_DIV;
	}
	if (PLL_OP_MPY != PLL_OP_MPY_PREV) {
		i2cRegWrite(0x30c, (uint8_t) (PLL_OP_MPY >> 8)); // total 11 bits
		i2cRegWrite(0x30d, (uint8_t) (PLL_OP_MPY & 0xff));
		PLL_OP_MPY_PREV = PLL_OP_MPY;

		// readback the just written value
		uint32_t value = i2cRegRead(0x30c) << 8;
		value |= i2cRegRead(0x30d);
		if (value != PLL_OP_MPY) {
			printf(
					"ERROR     : cam %d system 0x030c : 0x%04x (%4u) set PLL_OP_MPY (PLL output system multiplier), but should be 0x%04x\n",
					camIndex, value, value, PLL_OP_MPY);
		} else {
			printf("INFO      : cam %d system 0x030c : 0x%04x (%4u) set PLL_OP_MPY (PLL output system multiplier)\n",
					camIndex, value, value);
		}
		fflush(stdout);
	}

	if (testPattern != testPatternPrev) {
		printf("INFO      : cam %d system 0x0601 :   0x%02x (%4u) set test pattern\n", camIndex, testPattern,
				testPattern);
		fflush(stdout);
		i2cRegWrite(0x0601, (uint8_t) (testPattern & 0xff));
		testPatternPrev = testPattern;
	}
	if (analogGain != analogGainPrev) {
		printf("INFO      : cam %d system 0x0157 :   0x%02x (%4u) set analog gain\n", camIndex, analogGain, analogGain);
		fflush(stdout);
		i2cRegWrite(0x0157, (uint8_t) (analogGain & 0xff));
		analogGainPrev = analogGain;
	}
	if (shutter != shutterPrev) {
		printf("INFO      : cam %d system 0x015a : 0x%04x (%4u) set shutter\n", camIndex, shutter, shutter);
		fflush(stdout);
		i2cRegWrite(0x015a, (uint8_t) (shutter >> 8));
		i2cRegWrite(0x015b, (uint8_t) (shutter & 0xff));
		shutterPrev = shutter;
	}
	if (blackLevel != blackLevelPrev) {
		printf("INFO      : cam %d system 0xd1ea : 0x%04x (%4u) set blackLevel\n", camIndex, blackLevel, blackLevel);
		fflush(stdout);
		i2cRegWrite(0xd1ea, (uint8_t) (blackLevel >> 8));
		i2cRegWrite(0xd1eb, (uint8_t) (blackLevel & 0xff));
		blackLevelPrev = blackLevel;
	}

	if (lines != linesPrev) {
		printf("INFO      : cam %d system 0x0160 : 0x%04x (%4u) set amount of lines\n", camIndex, lines, lines);
		fflush(stdout);
		i2cRegWrite(0x0160, (uint8_t) (lines >> 8));
		i2cRegWrite(0x0161, (uint8_t) (lines & 0xff));
		linesPrev = lines;
	}

	uint16_t pixels = pixelsBase + pixelsOffset;
	if (pixels != pixelsPrev) {
		printf("INFO      : cam %d system 0x0162 : 0x%04x (%4u) set amount of pixels\n", camIndex, pixels, pixels);
		fflush(stdout);
		i2cRegWrite(0x0162, (uint8_t) (pixels >> 8));
		i2cRegWrite(0x0163, (uint8_t) (pixels & 0xff));
		pixelsPrev = pixels;
	}

	if (xStart != xStartPrev) {
		printf("INFO      : cam %d system 0x0164 : 0x%04x (%4u) set x start position\n", camIndex, xStart, xStart);
		fflush(stdout);
		i2cRegWrite(0x0164, (uint8_t) (xStart >> 8));
		i2cRegWrite(0x0165, (uint8_t) (xStart & 0xff));
		xStartPrev = xStart;
	}

	if (xEnd != xEndPrev) {
		printf("INFO      : cam %d system 0x0166 : 0x%04x (%4u) set x end position\n", camIndex, xEnd, xEnd);
		fflush(stdout);
		i2cRegWrite(0x0166, (uint8_t) (xEnd >> 8));
		i2cRegWrite(0x0167, (uint8_t) (xEnd & 0xff));
		xEndPrev = xEnd;
	}

	if (xSize != xSizePrev) {
		printf("INFO      : cam %d system 0x016c : 0x%04x (%4u) set x size\n", camIndex, xSize, xSize);
		fflush(stdout);
		i2cRegWrite(0x016c, (uint8_t) (xSize >> 8));
		i2cRegWrite(0x016d, (uint8_t) (xSize & 0xff));
		xSizePrev = xSize;
	}

	if (yStart != yStartPrev) {
		printf("INFO      : cam %d system 0x0168 : 0x%04x (%4u) set y start position\n", camIndex, yStart, yStart);
		fflush(stdout);
		i2cRegWrite(0x0168, (uint8_t) (yStart >> 8));
		i2cRegWrite(0x0169, (uint8_t) (yStart & 0xff));
		yStartPrev = yStart;
	}
	if (yEnd != yEndPrev) {
		printf("INFO      : cam %d system 0x016a : 0x%04x (%4u) set y end position\n", camIndex, yEnd, yEnd);
		fflush(stdout);
		i2cRegWrite(0x016a, (uint8_t) (yEnd >> 8));
		i2cRegWrite(0x016b, (uint8_t) (yEnd & 0xff));
		yEndPrev = yEnd;
	}
	if (ySize != ySizePrev) {
		printf("INFO      : cam %d system 0x016e : 0x%04x (%4u) set y size\n", camIndex, ySize, ySize);
		fflush(stdout);
		i2cRegWrite(0x016e, (uint8_t) (ySize >> 8));
		i2cRegWrite(0x016f, (uint8_t) (ySize & 0xff));
		ySizePrev = ySize;
	}
	if (imageOrientation != imageOrientationPrev) {
		printf("INFO      : cam %d system 0x016e : 0x%04x (%4u) set image orientation\n", camIndex, imageOrientation,
				imageOrientation);
		fflush(stdout);
		i2cRegWrite(0x0172, (uint8_t) (imageOrientation & 0xff));
		imageOrientationPrev = imageOrientation;
	}

}

static inline void raspiCameraManufacturerAccess() {
	// acquire access to the camera manufacturer specific registers of the camera (of which only the black level is used)
	i2cRegWrite(0x30eb, 0x05);
	i2cRegWrite(0x30eb, 0x0c);
	i2cRegWrite(0x300a, 0xff);
	i2cRegWrite(0x300b, 0xff);
	i2cRegWrite(0x30eb, 0x05);
	i2cRegWrite(0x30eb, 0x09);
}

static inline void raspiCameraGetTemperature() {
#if defined(__arm__) || defined(__aarch64__)
	// bit   7 : start sensor temperature measurement
	// bit 6-0 : sensor temperature
	uint16_t addr = 0x0140;
	if (cmosTempEnabled) {
		uint8_t value = 0x7f & i2cRegRead(addr); // strip of enable bit in case the latest measurement still pending
		// convert to degrees Celsius (see IMX219PQH datasheet)
		cmosTemp = (int8_t) round((value * 0.82) - 9.9);
		// printf("INFO      : cam %u system camera sensor temperature %2d 'C (value %3u)\n", camIndex, cmosTemp, value);
		// fflush(stdout);
	} else {
		cmosTemp = 0;
	}

	// start sensor temperature measurement for the next cycle
	i2cRegWrite(addr, (uint8_t) (1 << 7));
	cmosTempEnabled = true;
#else
	cmosTemp = 0;
#endif
}

static inline void statisticsCollect() {
	FILE *fp = NULL;
#if defined(__arm__) || defined(__aarch64__)
	// 0: under-voltage
	// 1: arm frequency capped
	// 2: currently throttled
	// 4: soft temperature limit (added in 2018)

	// 16: under-voltage has occurred
	// 17: arm frequency capped has occurred
	// 18: throttling has occurred
	// 19: soft temperature limit occurred (added in 2018)
	fp = popen("vcgencmd get_throttled", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u system cannot execute vcgencmd get_throttled command, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	uint32_t fullcpuStatus;
	// move bits 16-19 to 4-7 so the result fits in 8 bits instead of 32
	if( fscanf(fp, "throttled=0x%x", &fullcpuStatus) <= 0 ) {
		printf("ERROR     : cam %u system cannot extract CPU status from vcgencmd get_throttled, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	pclose(fp);
	cpuStatus = (uint8_t)( ( fullcpuStatus & 0x000f0000 ) >> 12 ) | ( fullcpuStatus & 0x0000000f );

	fp = popen("vcgencmd measure_temp", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u system cannot execute vcgencmd measure_temp command, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	float tmpFl = -1.0;
	// GPU temperature is provided in float 'C
	if( fscanf(fp, "temp=%f", &tmpFl) <= 0 ) {
		printf("ERROR     : cam %u system cannot extract GPU temperature from vcgencmd measure_temp, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	if ((tmpFl >= 0.0) && (tmpFl < 255.5)) {
		gpuTemp = (uint8_t) round(tmpFl);
	} else {
		gpuTemp = 255;
	}
	pclose(fp);
#else
	// no cpu status and relevant gpu temperature available on x86_64
	cpuStatus = 0;
	gpuTemp = 0;
#endif

	fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u system cannot open /sys/class/thermal/thermal_zone0/temp, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	int tmpInt = -1;
	// CPU temperature is provided as integer in milli 'C
	if (fscanf(fp, "%d", &tmpInt) <= 0) {
		printf(
				"ERROR     : cam %u system cannot extract CPU temperature from /sys/class/thermal/thermal_zone0/temp, message: %s\n",
				camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	tmpInt = tmpInt / 1000;
	if ((tmpInt >= 0) && (tmpInt < 256)) {
		cpuTemp = (uint8_t) tmpInt;
	} else {
		cpuTemp = 255;
	}
	fclose(fp);

	fp = fopen("/proc/loadavg", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u system cannot open /proc/loadavg, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	float tmpFl2 = -1.0;
	// average CPU load over one minute
	if (fscanf(fp, "%f", &tmpFl2) <= 0) {
		printf("ERROR     : cam %u system cannot extract average CPU load from /proc/loadavg, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	tmpFl2 = tmpFl2 * 32.0; // use fixed point range 0 to 255/32.0 = 7.97
	if ((tmpFl2 >= 0.0) && (tmpFl2 < 255.5)) {
		cpuLoad = (uint8_t) round(tmpFl2);
	} else {
		cpuLoad = 255;
	}
	fclose(fp);

	fp = fopen("/proc/uptime", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u system cannot open /proc/uptime, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	tmpFl2 = -1.0;
	// cpu uptime in seconds (with fraction)
	if (fscanf(fp, "%f", &tmpFl2) <= 0) {
		printf("ERROR     : cam %u system cannot extract CPU uptime from /proc/uptime, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	if ((tmpFl2 >= 0.0) && (tmpFl2 <= 0xffff)) {
		cpuUptime = (uint16_t) round(tmpFl2);
	} else {
		cpuUptime = 0xffff;
	}
	fclose(fp);

	char command[64];
	if (processId != 0) {
		sprintf(command, "ps -o etimes= -p %d", processId);
		fp = popen(command, "r");
		if (fp == NULL) {
			printf("ERROR     : cam %u system cannot execute %s, message: %s\n", camIndex, command, strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
		tmpInt = -1;
		// system application uptime in seconds
		if (fscanf(fp, "%d", &tmpInt) <= 0) {
			printf("ERROR     : cam %u system cannot system application uptime from ps -o etimes= -p %d, message: %s\n",
					camIndex, processId, strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
		if ((tmpInt >= 0) && (tmpInt <= 0xffff)) {
			systemApplicationUptime = (uint16_t) tmpInt;
		} else {
			systemApplicationUptime = 0xffff;
		}
		pclose(fp);
	} else {
		systemApplicationUptime = 0xffff;
	}

	raspiCameraGetTemperature();

	// printf("INFO      : cam %u system vcgencmd cpu status 0x%02x, GPU temp %u CPU temp %u uptime %4u\n", camIndex,
	//		cpuStatus, gpuTemp, cpuTemp, cpuUptime);
	// fflush(stdout);
}

static inline void waitForGrabberProcess(int uptime) {

// The camera i2c is shared between the "GPU" i2c interface and the GPIO pins available through /dev/i2c-0.
// During initialization the "GPU" configures the camera through the "GPU" i2c interface.
// This raspiSystem application uses the /dev/i2c-0 to configure the camera i2c.
// To prevent access conflicts, which results in hangs, wait until the "GPU" is ready
// with the i2c, before using it in this application

// TODO: replace following if for a way to determine /dev/video0 is used on raspi
#if !defined(__aarch64__)
	FILE *fp;
	char var[40];
	int grabberUptime = 0;
	bool waitForGrabber = true;

	while (waitForGrabber) {
#if defined(__arm__)
		fp = popen("ps -p `pidof raspiGrab` -o etimes= 2>/dev/null || echo -1", "r"); // return -1 when raspiGrab is not running at all
#else
		// no grabber is running on x86_64, for testing use the raspiSystem process uptime as raspiGrabber uptime
		if (valgrind) {
			// when using valgrind, use the pid of valgrind instead of raspiSystem
			fp = popen("ps -p `pidof valgrind.bin` -o etimes= 2>/dev/null || echo -1", "r"); // return -1 when valgrind is not running at all
		} else {
			// there might be running multiple raspiSystem processes on x86_64
			fp = popen("ps -p `pidof raspiSystem | awk '{print $1}'` -o etimes= 2>/dev/null || echo -1", "r"); // return -1 when raspiSystem is not running at all
		}
#endif
		if (fp == NULL) {
			printf(
					"ERROR     : cam %u system cannot execute ps -p `pidof raspiGrab` -o etimes= 2>/dev/null || echo -1, message: %s\n",
					camIndex, strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		}
		if (fgets(var, sizeof(var), fp) != NULL) {
			grabberUptime = atoi(var);
			if (grabberUptime < 0) {
				printf("INFO      : cam %d system grabber process not running (wait)\n", camIndex);
				fflush(stdout);
				sleep(1);
			} else if (grabberUptime < uptime) {
				printf("INFO      : cam %d system grabber process running for %d seconds (wait)\n", camIndex,
						grabberUptime);
				fflush(stdout);
				sleep(1);
			} else {
				printf("INFO      : cam %d system grabber process running for %d seconds (continue)\n", camIndex,
						grabberUptime);
				fflush(stdout);
				waitForGrabber = false;
			} // grabberUptime
		} else {
			printf("ERROR     : cam %d system return value from ps -p cannot be read, message %s\n", camIndex,
					strerror(errno));
			fflush(stdout);
			exit(EXIT_FAILURE);
		} // fgets
		pclose(fp);
	}
#else
	printf("WARNING   : cam %d system replace grabber by /dev/video0\n", camIndex );
	(void) uptime;
#endif

}

// compress the rgb image and send to x86_64
// the avaialble compression tool on raspi does not accept rgb as input, but does accept bmp
// so first convert rgb to bmp and then convert the bmp to jpg (with usage of the external program)
// Note: the raspi does not have the (turbo) jpg library default installed, so making directly
// use of the jpeg library would require all raspi boards to be update, so use exernal program instead
void doSendFrame(uint32_t frameCounter) {
	char bmpFileName[256];
	char jpgFileName[256];

	#if defined(__arm__) || defined(__aarch64__)
	strcpy(bmpFileName, "/dev/shm/raspiGrab.bmp");
	strcpy(jpgFileName, "/dev/shm/raspiGrab.jpg");
#else
	sprintf(bmpFileName, "/tmp/raspiGrab%u.bmp", camIndex);
	sprintf(jpgFileName, "/tmp/raspiGrab%u.jpg", camIndex);
#endif

	rgbToBmp(camIndex, grabFileName, bmpFileName, false, false);

	char command[1024];
	uint8_t compressionQuality = 85; // (~70kB), default 75 (~62kB)
	sprintf(command, "cjpeg -optimize -quality %u -outfile %s %s", compressionQuality, jpgFileName, bmpFileName);

	if (system(command) < 0) {
		printf("ERROR     : cam %u system shell cannot execute %s, message: %s\n", camIndex, command, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
	imageSend(jpgFileName, frameCounter, false);
}

#if defined(__arm__) || defined(__aarch64__)
// sendFrame implementation for arm
void* sendFrame(void *arg) {
	(void) arg;

	uint32_t frameCounterNext = 0;

	int shmFd = shm_open("/raspiGrab", O_RDONLY, 0666);
	if (shmFd < 0) {
		printf("ERROR     : cam %u system cannot open /dev/shm/raspiGrab with shm_open for reading, message %s\n",
				camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

	uint8_t *shmData = 0;
#define SHM_SIZE ( 3 * ROI_WIDTH * ROI_HEIGHT ) // 3 bytes per pixel
	shmData = (uint8_t *) mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shmFd, 0);
	printf("INFO      : cam %u system shared memory mapped to address: %p\n", camIndex, shmData);

	while (keepRunning) {

		// determine if there is new data in the shared memory buffer
		uint32_t *frameCounterPtr = (uint32_t *) shmData;
		if (*frameCounterPtr != frameCounterPrev) {
			frameCounterPrev = *frameCounterPtr;
			if (frameCounterPrev % 40 == 0) {
				// printf("INFO      : cam %u system frameCounter %u\n", camIndex, frameCounterPrev);
				fflush(stdout);
				if (frameCounterNext != frameCounterPrev) {
					// if the sending of the previous image was not in time
					printf("WARNING   : cam %u system was not able to store frameCounter %u (system load)\n", camIndex,
							frameCounterNext);
					fflush(stdout);

				}
				doSendFrame(frameCounterPrev);
				frameCounterNext = frameCounterPrev + 40;
			}

		} else {
			// wait and poll again
			usleep(1000);// the period is 25ms, so time enough to copy the frame
		} // if (*frameCounterPtr != frameCounterPrev)
	} // while (keepRunning)

	munmap(shmData, SHM_SIZE);
	close(shmFd);
	return NULL;
}
#else

// sendFrame implementation for x86_64
void* sendFrame(void *arg) {
	(void) arg;

	while (keepRunning) {
		// TODO: run this loop every second
		doSendFrame(frameCounterPrev);
		frameCounterPrev += 40;
	} // while (keepRunning)

	return NULL;
}

#endif

void* raspiConfig(void *arg) {
	(void) arg;

	raspiCameraGetIdentifiers();
// raspiCameraConfigDump();
	raspiCameraManufacturerAccess();
	raspiCameraConfigUpdate();
// raspiCameraConfigDump();
	while (keepRunning) {
		struct sockaddr_in fromAddr;
		socklen_t fromAddrlen = sizeof(fromAddr);
		packetT rxPacket;
		// block until data received
		ssize_t nBytes = recvfrom(multReceiveFd, &rxPacket, sizeof(rxPacket), 0, (struct sockaddr *) &fromAddr,
				&fromAddrlen);
		if (nBytes < 0) {
			printf("ERROR     : cam %u system reading from socket, %s", camIndex, strerror(errno));
			fflush(stdout);
			close(multReceiveFd);
			exit(EXIT_FAILURE);
		}

		bool valid = true;
		if (rxPacket.size != nBytes) {
			printf("ERROR     : cam %u system packet incomplete, received %zd bytes, but expected %u bytes\n", camIndex,
					nBytes, rxPacket.size);
			fflush(stdout);
			valid = false;
		} else {
			//printf("INFO      : cam %u system received %u bytes from sender %s:%u\n", camIndex, rxPacket.size, inet_ntoa(fromAddr.sin_addr),
			//		ntohs(fromAddr.sin_port));
		}

		if (valid) {
			if (rxPacket.id == 64) {
				size_t expected = 52;
#ifdef WHITE_BLACK_BALL_SEARCH
				expected += 10;
#endif
				if (rxPacket.size != expected) {
					printf(
							"ERROR     : cam %u system configuration packet should be %zu bytes, but received %u bytes\n",
							camIndex, expected, rxPacket.size);
					fflush(stdout);
				} else {
					sendConfigAcknowledge(); // used to measure the UDP round trip

					size_t ii = 0;
					EXCK_FREQ = rxPacket.pl.u16[ii++];
					VTPXCK_DIV = rxPacket.pl.u16[ii++];
					VTSYCK_DIV = rxPacket.pl.u16[ii++];
					PREPLLCK_VT_DIV = rxPacket.pl.u16[ii++];
					PREPLLCK_OP_DIV = rxPacket.pl.u16[ii++];
					PLL_VT_MPY = rxPacket.pl.u16[ii++];
					OPPXCK_DIV = rxPacket.pl.u16[ii++];
					OPSYCK_DIV = rxPacket.pl.u16[ii++];
					PLL_OP_MPY = rxPacket.pl.u16[ii++];
					cameraReset = rxPacket.pl.u16[ii++];
					cameraVerbose = rxPacket.pl.u16[ii++];
					testPattern = rxPacket.pl.u16[ii++];
					analogGain = rxPacket.pl.u16[ii++];
					shutter = rxPacket.pl.u16[ii++];
					blackLevel = rxPacket.pl.u16[ii++];
					lines = rxPacket.pl.u16[ii++];
					pixelsBase = rxPacket.pl.u16[ii++];
					xStart = rxPacket.pl.u16[ii++];
					xEnd = rxPacket.pl.u16[ii++];
					xSize = rxPacket.pl.u16[ii++];
					yStart = rxPacket.pl.u16[ii++];
					yEnd = rxPacket.pl.u16[ii++];
					ySize = rxPacket.pl.u16[ii++];
					imageOrientation = rxPacket.pl.u16[ii++];

					// only reset on 0 to 1 transfer of cameraReset
					if ((cameraReset == 1) && (cameraResetPrev != 1)) {
						printf("INFO      : cam %d system camera reset requested\n", camIndex);
						fflush(stdout);

						i2cRegWrite(0x0103, 0x01); // software reset
						// direct access after software reset causes deadlock
						usleep(100);
						raspiCameraManufacturerAccess();

						i2cRegWrite(0x0114, 0x01); // CSI_LANE_MODE 2 lanes
						i2cRegWrite(0x0128, 0x00); // DPHY_CTRL : MIPI global timing setting auto mode

						EXCK_FREQ_PREV = 0xffff; // 0x012a-0x012b
						VTPXCK_DIV_PREV = 0xffff; // 0x0301
						VTSYCK_DIV_PREV = 0xffff; // 0x0303
						PREPLLCK_VT_DIV_PREV = 0xffff; // 0x0304
						PREPLLCK_OP_DIV_PREV = 0xffff; // 0x0305
						PLL_VT_MPY_PREV = 0xffff; // 0x0306-0x0307
						OPPXCK_DIV_PREV = 0xffff; // 0x0309
						OPSYCK_DIV_PREV = 0xffff; // 0x030b
						PLL_OP_MPY_PREV = 0xffff; // 0x030c-0x030d

						// camera control and white balance (software)
						testPattern = TEST_PATTERN;
						analogGain = ANALOG_GAIN;
						shutter = SHUTTER; // 0x015a, course integration time, 16 bits
						blackLevel = BLACK_LEVEL; // 0xd1ea, 10 bits

						cameraResetPrev = CAMERA_RESET;
						testPatternPrev = 0xffff;
						analogGainPrev = 0xffff;
						shutterPrev = 0xffff;
						blackLevelPrev = 0xffff;

						// camera dimensions
						lines = LINES; // 0x0160-0x0161
						pixelsBase = PIXELS; // 0x0162-0x0163
						pixelsOffset = 0;
						xStart = X_START; // 0x0164-0x0165
						xEnd = X_END; // 0x0166-0x0167
						xSize = X_SIZE; // 0x016c-0x016d
						yStart = Y_START; // 0x0168-0x0169
						yEnd = Y_END; // 0x016a-0x016b
						ySize = Y_SIZE; // 0x016e-0x016f

						linesPrev = 0xffff; // 0x0160-0x0161
						pixelsPrev = 0xffff; // 0x0162-0x0163
						xStartPrev = 0xffff; // 0x0164-0x0165
						xEndPrev = 0xffff; // 0x0166-0x0167
						xSizePrev = 0xffff; // 0x016c-0x016d
						yStartPrev = 0xffff; // 0x0168-0x0169
						yEndPrev = 0xffff; // 0x016a-0x016b
						ySizePrev = 0xffff; // 0x016e-0x016f
						imageOrientationPrev = 0xffff; // 0x0172

						raspiCameraConfigUpdate();

#ifdef x4_BINNING
						i2cRegWrite(0x0174, 0x02); // BINNING_MODE_H_A [1:0] : H-direction x4-binning
						i2cRegWrite(0x0175, 0x02);// BINNING_MODE_V_A [1:0] : V-direction x4-binning
#else
#ifdef X2_ANALOG_SPECIAL
						i2cRegWrite(0x0174, 0x03); // BINNING_MODE_H_A [1:0] : H-direction x2-analog special binning
						i2cRegWrite(0x0175, 0x03);// BINNING_MODE_V_A [1:0] : V-direction x2-analog special binning
#else
						i2cRegWrite(0x0174, 0x01); // BINNING_MODE_H_A [1:0] : H-direction x2-binning
						i2cRegWrite(0x0175, 0x01); // BINNING_MODE_V_A [1:0] : V-direction x2-binning
#endif
#endif

						i2cRegWrite(0x0176, 0x00); // BINNING_CAL_MODE_H_A [0] : H-direction average mode binning (instead of sum)
						i2cRegWrite(0x0177, 0x00); // BINNING_CAL_MODE_V_A [0] : V-direction average mode binning (instead of sum)
						i2cRegWrite(0x0100, 0x01); // streaming mode
					} else {
						raspiCameraConfigUpdate();
					}
					cameraResetPrev = cameraReset;

					// only verbose on 0 to 1 transfer of cameraVerbose
					if ((cameraVerbose == 1) && (cameraVerbosePrev != 1)) {
						raspiCameraConfigDump();
					}
					cameraVerbosePrev = cameraVerbose;

				}
			} else if (rxPacket.id == 65) {
				if (rxPacket.pl.u32[0] == 0xdead0011) {
					printf("WARNING   : cam %u system received poweroff\n", camIndex);
					fflush(stdout);
					if (system("sudo /sbin/poweroff") <= 0) {
						printf("ERROR     : cam %u system cannot execute system \"sudo /sbin/poweroff\", message: %s\n",
								camIndex, strerror(errno));
						fflush(stdout);
						exit(EXIT_FAILURE);
					}

				}
			} else if (rxPacket.id == 66) {
				if (rxPacket.pl.u32[0] == 0xdead0022) {
					printf("WARNING   : cam %u system received reboot\n", camIndex);
					sleepKillReboot(1);
				}
			} else if (rxPacket.id == 67) {
				size_t expected = CAM_PACKET_HEADER_SIZE + 4; // one bye for each camera
				if (rxPacket.size != expected) {
					printf(
							"ERROR     : cam %u system configuration pixel offset list should be %zu bytes, but received %u bytes\n",
							camIndex, expected, rxPacket.size);
					fflush(stdout);
				} else {
					pixelsOffset = rxPacket.pl.s8[camIndex]; // the 4 bytes in the payload each relate to one camera
					raspiCameraConfigUpdate();
				}
			} else if (rxPacket.id == 68) {
				if (rxPacket.pl.u32[0] == 0xdead0099) {
					printf("WARNING   : cam %u system received exit application\n", camIndex);
					fflush(stdout);
					keepRunning = false;
				}
			} else if (rxPacket.id == 69) {
				// packet to synchronize frame counter of cam1, cam2 and cam3 with frame counter of cam0
			} else {
				if (rxPacket.id >= 69 && rxPacket.id < 80) {
					printf("ERROR     : cam %u system received invalid system packet id %u\n", camIndex, rxPacket.id);
				}
			} // if (rx.packet.id
		} // if (valid)
	} // while (keepRunning)

	return NULL;
}

int main(int argc, char** argv) {

	int opt = 0;
	int runTime = -1;

	while ((opt = getopt(argc, argv, "g:hl:t:v")) != -1) {
		switch (opt) {
		case 'h':
			printf("INFO      : -l camera index\n");
			printf("          : -t run time in seconds (optional)\n");
			return 0;
			break;
		case 'g':
			strcpy(grabFileName, optarg);
			break;
		case 'l':
			camIndex = atoi(optarg);
			break;
		case 't':
			runTime = atoi(optarg);
			break;
		case 'v':
			valgrind = true;
			break;
		}
	}

	printf("INFO      : cam %u system start\n", camIndex);

	printf("INFO      : cam %u system grabber file %s\n", camIndex, grabFileName);

// get process id which is used to determine raspi system application uptime
	processId = getpid();
	printf("INFO      : cam %u system process ID %d\n", camIndex, processId);

// collect the md5sum of all relevant files, which is used by the x86_64 to determine if the software version is correct
	FILE *fp = fopen("md5sumAllFiles.log", "r");
	if (fp == NULL) {
		printf("ERROR     : cam %u system cannot open md5sumAllFiles.log, message: %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (fscanf(fp, "%8x%8x%8x%8x", &md5sumPart0, &md5sumPart1, &md5sumPart2, &md5sumPart3) <= 0) {
		printf("ERROR     : cam %u system cannot extract values from md5sumAllFiles.log, message: %s\n", camIndex,
				strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	fclose(fp);
	printf("INFO      : cam %u system md5sumAlFiles %08x%08x%08x%08x\n", camIndex, md5sumPart0, md5sumPart1,
			md5sumPart2, md5sumPart3);

	if (pthread_mutex_init(&sendMutex, NULL) != 0) {
		printf("ERROR     : cam %u system cannot initialize send mutex, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	multiCastSendSetup();
// synchronize packet counter with CPU
// TODO: send message to x86_64 to indicate grabber process just started
	packetCntSyncSend();

	multiCastReceiveSetup();

	printf("INFO      : cam %u system wait for grabber process\n", camIndex);
// in most cases there is no i2c conflict when the grabber is running for 1 second, add 1 more second for headroom
	waitForGrabberProcess(2);

	if (pthread_mutex_init(&i2cMutex, NULL) != 0) {
		printf("ERROR     : cam %u system cannot initialize i2c mutex, message: %s\n", camIndex, strerror(errno));
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

#if defined(__arm__) || defined(__aarch64__)
	i2cFd = open("/dev/i2c-0", O_RDWR); // returns -1 if an error occured
	if (i2cFd < 0) {
		printf("ERROR     : cam %u system cannot open /dev/i2c-0, message: %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	} else {
		printf("INFO      : cam %u system use /dev/i2c-0 to control camera\n", camIndex);
	}

#endif
	int err = pthread_create(&(sendFrameThreadId), NULL, &sendFrame, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u system cannot start sendFrame thread, %s\n", camIndex, strerror(err));
	}

	err = pthread_create(&(raspiConfigThreadId), NULL, &raspiConfig, NULL);
	if (err != 0) {
		printf("ERROR     : cam %u system cannot start raspiConfig thread, %s\n", camIndex, strerror(err));
	}

// setup done
	while (keepRunning) {
		// TODO: run this loop every second
		md5sumSend();
		statisticsCollect();
		statisticsSend();
		backWardsCompatibleStaticsSends();

		if (runTime == 0) {
			keepRunning = false;
		}
		if (runTime > 0) {
			runTime--; // decrease every second
		}
		sleep(1);
	}

// gracefully shutdown for valgrind report

// nothing is send anymore to the x86_64
	multiCastSendClose();

#if ! defined(__arm__)
// send a stop command to the still running configuration receive thread
// only for valgrind graceful shutdown on x86_64
	printf("INFO      : cam %u system send stop to configuration receive thread\n", camIndex);
	multiCastReceiveLoopBackSetup();
	multiCastReceiveLoopBackStop();
	multiCastReceiveLoopBackClose();
#endif

// wait until the sendFrameThread thread has been stopped
	pthread_join(sendFrameThreadId, NULL);

// stop the configuration thread
// wait until the raspi configuration thread has been stopped
	pthread_join(raspiConfigThreadId, NULL);

// now we also can close the receive configuration port
	multiCastReceiveClose();

#if defined(__arm__) || defined(__aarch64__)
	close(i2cFd);
#endif

	pthread_mutex_destroy(&sendMutex);
	pthread_mutex_destroy(&i2cMutex);

	printf("INFO      : cam %u system all done\n", camIndex);
	return EXIT_SUCCESS;
}
