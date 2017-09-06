 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <cerrno>
#include <fcntl.h> // for O_RDWR
#include <string.h>
#include <sys/ioctl.h> // for dtr control

#include "ttyUsb.hpp"


using namespace std;

serial::serial( ) {
	init( "___ ", "___ ", "not set", false );
}

serial::~serial( ) {
	close(port);
}

void serial::init( std::string rPrefix, std::string sPrefix, std::string name, bool verbose ) {
	port = -1;
	index = -1;
	this->rPrefix = rPrefix;
	this->sPrefix = sPrefix;
	this->name = name;
	if( verbose ) { printf("%.3s  INFO    : %.10s initialized\n", sPrefix.c_str(), name.c_str() ); }

	txPacketAckId = 0; // first packet start with ackId 0, then increment (wrap around) for each following packet
 	
	// used in receivedBytes
	state = 0; // state 0 = search SOF, state 1 = complete the packet
	rxBytes = 0; // current packet byte size
	rxPacketTmpPointer = 0; // for efficient iterating through the rx packet
	
	receivedPacketsCnt = 0;
	receivedPacketsCntPrev = 0;
	receivedPacketsCntPrevSend = 0;
	receiveChecksumFail = 0;
	receiveAckIdMis = 0;
	receiveTimeOut = false;

	receiveReadError = false;
	sendWriteError = false;
	sendStopped = false;
	farEndBufferSize = 0;
	ackId = 0;
	initialized = 0;
	sendPacketsCnt = 0;
	primarySetPoint = 0;; // keep track of the setPoint that was send to the board for printing

	farEndAckIdMis = 0;
	farEndChecksumFail = 0;
	farEndPacketOverwritten = 0;
	farEndBufferOverflow = 0;
	farEndWrongPayloadSize = 0;

	sendError = false; // used to exchange status between receiver thread and sender thread
	receiveError = false; // used to exchange status between receiver thread and sender thread
	deviceErrorThreshold = 0;
	deviceErrorPrintCount = 0;
	deviceOpen = false; // handshake for the receive thread to inform the send thread the device open

	applicationId = 255;

	mode = MODE_AA_DO_NOT_USE_ZERO;
	angleZero = 0;
	tachoZero = 0;
	boardConfigured = false;

	sofAlignmentCounter = 0;

	rxPacketTmp.sof = SOF; // should never be overwritten
	rxPacketTmp.ackId = 0xff;
	rxPacketTmp.feedbackId = 0xff;
	rxPacketTmp.bufferSpace = 0;
	rxPacketTmp.responseType = 0xff;
	rxPacketTmp.checksum = 0xff; // is overwritten when receiving packet, but never read
	rxPacketTmp.payloadSize = 0;
	rxPacketTmp.endCheck = 0x98; // should never be overwritten
}

// Send packet to board
void serial::sendPacket( toBoardPacketT txPacket ) {
 	int packetSize = txPacket.payloadSize + TO_BOARD_PACKET_HEADER_SIZE;
 	// because of the asynchronous communication we do not now the exact far end rx buffer size
 	// for now assume there are no more then 200 bytes in the pipeline
 	// TODO: add flow control
	size_t timeoutCounter = 0;
 	while( ( sendError == false ) && ( packetSize > ( farEndBufferSize - 128 ) ) ) { // the receive buffer of the board is 255 bytes
 		if( timeoutCounter > 10000 ) { // abort if no communication after 100ms
 			sendError = true;
 		} else {
 			timeoutCounter++;
 		}
 		usleep(10); // give the receive thread time to update the farEndBufferSize
 	}

 	if( sendError == false ) {
 	 	// Calculate the checksum
 		txPacket.sof = SOF;
 		txPacket.checksum = 0; // for new packet, checksum starts at 0
 		txPacket.ackId = txPacketAckId;
 		char checksum = 0;
 		char *txPacketPointer = (char *) &txPacket; // use char pointer to index the individual bytes of the txPacket
 		for( int ii = 0; ii < packetSize; ii++ ) { // iterate through all the bytes including the checksum itself (which was reset to 0)
 			checksum += *txPacketPointer; // use simple add as checksum (instead of e.g. crc8)
 			txPacketPointer++; // set the pointer to the next byte of the txPacket
 		}
 		txPacket.checksum = checksum; // store the calculated checksum in the packet

 		// send packet to the far end
 		sendPacketsCnt++;
 		int numBytes = write( port, &txPacket, packetSize );

 		if( numBytes == -1 ) {
 			printf( "%s WARNING : %s cannot write to device /dev/ttyUSB%d (probably device disconnected)\n", sPrefix.c_str(), name.c_str(), index );
 			sendError = true;
 			sendWriteError = true;
 		} else if( numBytes == 0 ) {
 			printf( "%s ERROR   : %s only 0 bytes written to device /dev/ttyUSB%d, abort\n", sPrefix.c_str(), name.c_str(), index );
 			sendError = true;
 		} else if( numBytes != packetSize ) {
 			printf( "%s ERROR   : %s %d bytes written instead of %d bytes\n", sPrefix.c_str( ), name.c_str(), numBytes, packetSize);
 			sendError = true;
 		} else {
 			// store the primary set point that has been send to the board so it can be printed in the receiver (to compare with the actual value)
 			if( txPacket.command == CMD_SET_PRIMARY_SETPOINT ) { primarySetPoint = txPacket.pl.s16[0]; }

 			motor.printSendPacket( sPrefix, txPacket );
 		}

 		// increment ackId for next packet (wrap around at 0xff)
 		txPacketAckId++;
 	}
}

// Receive a number of bytes from the serial input buffer
// The maximal size per read is the maximal packet size
// However most packets will be smaller then the maximal packet size, so probably
// multiple packets will be collected with a single read to the serial port
// This function blocks until data is available on the serial port
// If data is received it first start searching for the start of frame (= begin of packet)
// Then it will continue with adding bytes to the packet until the amount bytes is the same as the
// packet size (header + payload size) provided from the far end (board).
// When the packet is complete the processReceivedPacket function will be called.
// If there was more data read then for the packet, a search for the next start of frame will start
// to assemble the next packet.
int serial::receiveBytes( bool init, bool verbose ) {
	// TODO: packet rxPacketTmp should be local to this function, however then a initialization solution has to be found

	// get a number of bytes from the input buffer
	unsigned char rxBuffer[1024];
	int numBytes = read(port, &rxBuffer, FROM_BOARD_PACKET_SIZE);
	if( numBytes == -1 ) {
		receiveReadError = true;
		if( ! init ) {
			printf( "%s WARNING : %s cannot read from device /dev/ttyUSB%d (probably device disconnected)\n", rPrefix.c_str(), name.c_str(), index );
			fflush( stdout );
			receiveError = true;
		}
	} else if( numBytes == 0 ) {
		receiveTimeOut = true;
		if( ! init ) {
			printf( "%s ERROR   : %s only 0 bytes read from device /dev/ttyUSB%d (probably device disconnected or EMO activated)\n", rPrefix.c_str(), name.c_str(), index );
			fflush( stdout );
			receiveError = true;
		}
	} else if( numBytes > 0 ) {
		// process byte for byte, first search for the sof, then complete the packet
		for( int ii = 0; ii < numBytes; ii++ ) {

			switch (state){
			case 0: // state: search SOF
				if( rxBuffer[ii] == SOF ){
					state = 1; // sof received, now start filling the buffer
					// Note: do not need to store sof in rxPacketTmp because it will always be the same
					rxPacketTmpPointer = (char *) &rxPacketTmp; // use char pointer to index the individual bytes of the rxPacketTmp
					rxPacketTmpPointer++; // move pointer to the next element in rx packet because first byte (sof) stored
					rxBytes = 1; // already 1 byte received
				} else {
					sofAlignmentCounter++; // monitor how many bytes have been processed before the start of frame was found
				}
				break;

			case 1:	// state: complete the packet
				rxBytes++; // increment amount of received bytes
				if( rxBytes <= FROM_BOARD_PACKET_SIZE ) {
					*rxPacketTmpPointer = rxBuffer[ii]; // store received byte in rx packet
					rxPacketTmpPointer++; // move pointer to the next location of the rx packet (to store the next byte)
					if( rxBytes >= 4 ) { // TODO: replace by e.g. (rxPacketTmp.payloadSize-rxPacketTmp.sof)
						// size of payload (provided by far end) is available
						if( rxBytes == ((uint16_t)rxPacketTmp.payloadSize) + FROM_BOARD_PACKET_HEADER_SIZE ) {
							// received the last byte of current packet, packet complete, all done
							memcpy((void *) &rxPacket, (void *) &rxPacketTmp, rxPacketTmp.payloadSize + FROM_BOARD_PACKET_HEADER_SIZE );
							processReceivedPacket( );
							state = 0; // start searching for the next packet
							if( rxPacketTmp.endCheck != 0x98 ) { // check if rx buffer was written out of range
								printf( "%s ERROR   : overwritten end of rx packet\n", rPrefix.c_str( ));
								exit( EXIT_FAILURE );
							}
						} else if( rxBytes > ((uint16_t)rxPacketTmp.payloadSize) + FROM_BOARD_PACKET_HEADER_SIZE ) {
							printf("%s ERROR   : more bytes received then set by the far end through the payload size\n", rPrefix.c_str( ));
							exit( EXIT_FAILURE );
						}
					}
				} else {
					printf("%s ERROR   : rx packet (buffer) full, should never happen\n", rPrefix.c_str( ) );
					exit( EXIT_FAILURE );
				}
				break;
			default:
				break;
			}
		} // for ii < numBytes
	} else {
		printf( "%s ERROR   : unexpected number of bytes %d\n", rPrefix.c_str(), numBytes );
		fflush( stdout );
	}
	fflush( stdout );
	return numBytes;
}


// process the received package
void serial::processReceivedPacket( ) {
	rxPacketAckIdCheck( );
	if( rxPacketChecksumCheck( ) ) {
		// ignore packets with incorrect checksum
		receivedPacketsCnt++; // only packets with valid checksum are counted

		// tell the sender how much space is available in the board
		farEndBufferSize = rxPacket.bufferSpace;

		if( rxPacket.responseType == RESP_ERROR ) {
			// update statistics
			uint16_t commError = rxPacket.pl.u16[0];
			if( ( commError & COMMUNICATION_ERROR_TO_BOARD_CHECKSUM) != 0 ) { farEndChecksumFail++; }
			if( ( commError & COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC) != 0 ) { farEndAckIdMis++; }
			if( ( commError & COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN) != 0 ) { farEndPacketOverwritten++; }
			if( ( commError & COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW) != 0 ) { farEndBufferOverflow++; }
			if( ( commError & COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE) != 0 ) { farEndWrongPayloadSize++; }
		} else if ( rxPacket.responseType == RESP_MODE ) {
			// provide mode for the sender to initialize the board (e.g. configure driver when motor is disabled), TODO: this can be moved to high level communication
			mode = rxPacket.pl.u8[0];
		} else if ( rxPacket.responseType == RESP_ANGLE_TACHO_ZERO ) {
			// provide the angle zero and tacho zero for the sender, TODO: this can be moved to high level communication
			angleZero = rxPacket.pl.u16[0];
			tachoZero = rxPacket.pl.u16[1];
		}

		// provide the packet to communication, if needed remove old packets from the queue (in case communication did not process them)
		if( fromBoardPacket.size( ) > 255 ) { fromBoardPacket.pop( ); }
	    fromBoardPacket.push( rxPacket );
	}
}

// check if still packets are received
void serial::checkReceive( ) {
	if( receivedPacketsCnt == receivedPacketsCntPrev ) {
		deviceErrorThreshold++; // packet counter did not update since last check
	} else {
		deviceErrorThreshold = 0;
	}
	receivedPacketsCntPrev = receivedPacketsCnt;

	if( deviceErrorThreshold > 1000 ) {
		// assume the endpoint is not responding anymore
		receiveError = true;
		printf("%s ERROR   : it has been a long time before the last packet was received\n", rPrefix.c_str( ) );
	}
}

void serial::sendClearAllErrors( ) {
	toBoardPacketT tx;
	tx.command = CMD_CLEAR_ERRORS;
	tx.payloadSize = 14;
	for( int ii = 0; ii < tx.payloadSize; ii++ ){
		tx.pl.u8[ii] = 0xff;
	}
	sendPacket( tx );
}

void serial::sendClearKnownErrors( fromBoardPacketT rx ) {
	toBoardPacketT tx;
	tx.command = CMD_CLEAR_ERRORS;
	tx.payloadSize = 14;
	for( int ii = 0; ii < 7; ii++ ){
		tx.pl.u16[ii] = rx.pl.u16[ii];
	}

	// limit the queue depth and if needed throw old ones away
	if( toBoardPacket.size( ) > 255 ) { toBoardPacket.pop( ); }
	// store the new packet so it can be send by the send thread
    toBoardPacket.push( tx );
}

// Check if calculated packet checksum is the same as the checksum provided by the far end
bool serial::rxPacketAckIdCheck( ) {
	bool valid = true;
	if( initialized > 2 ) {
		// sometimes 2 packets are missed before in sync, maybe there is an overwrite in the receive buffer at startup
		if( ackId != rxPacket.ackId ) {
			valid = false;
			printf("%s ERROR   : packet ack id is 0x%02x but should be 0x%02x\n", rPrefix.c_str( ), rxPacket.ackId, ackId);
			receiveAckIdMis++;
		}
	} else {
		initialized++;
	}
	ackId = rxPacket.ackId + 1;
	return valid;
}

bool serial::rxPacketChecksumCheck( ) {
	bool valid = true;
	uint8_t checksum = 0;
	char *rxPacketPointer = (char *) &rxPacket; // use char pointer to index the individual bytes of the rxPacket
	for( int ii = 0; ii < ( FROM_BOARD_PACKET_HEADER_SIZE + rxPacket.payloadSize ); ii++ ) {
		checksum += *rxPacketPointer;
		rxPacketPointer++;
	}
	checksum -= rxPacket.checksum; // checksum added in above loop, but checksum itself should be excluded from check calculation

	if( checksum != rxPacket.checksum ) {
		valid = false;
		printf("%s ERROR   : checksum error on this received package:\n", rPrefix.c_str( ));

		motor.printReceivedPacket( rPrefix, rxPacket, mode, primarySetPoint, applicationId );

		printf("%s ERROR   : packet checksum is 0x%02x but calculated checksum is 0x%02x\n", rPrefix.c_str( ), rxPacket.checksum, checksum);
		receiveChecksumFail++;
	}
	return valid;
}


void serial::openDevice( speed_t speed, tcflag_t cflag ) {
	struct termios tio;

	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag = cflag;
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 0; // Minimum number of characters for noncanonical read (MIN).
	tio.c_cc[VTIME] = 3; // Timeout in deciseconds for noncanonical read (TIME) 1 = 300ms (keep low to prevent waiting for data from compass)

	char portName[256];
	sprintf(portName, "/dev/ttyUSB%d", index );

	int speedInt = 0;
	if( speed == B9600 ) { speedInt = 9600; }
	else if( speed == B115200 ) { speedInt = 115200; }

	string conf = "XXX";
	if( cflag == ( CS8 | CSTOPB | CREAD | CLOCAL ) ) {
		conf = "8n2";
	} else if( cflag == ( CS8 | CREAD | CLOCAL ) ) {
		conf = "8n1";
	}

	printf("%.3s  INFO    : open device %s %d %s\n", rPrefix.c_str( ), portName, speedInt, conf.c_str() );
	port = open(portName, O_RDWR ); // use global variable (in the C++ this will be performed in the constructor)

	// do not use O_NONBLOCK, because we want to wait for a reply until the VTIME expires
	// this prevents adding a nanosleep between the write and read
    // nanosleep((struct timespec[]){{0, 10*1000L}}, NULL); /* 1000 us */
	// O_NONBLOCK is posix-specified name for O_NDELAY
	// O_NOCTTY If set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	cfsetospeed(&tio, speed);
	cfsetispeed(&tio, speed);

	tcsetattr(port, TCSANOW, &tio);
	deviceOpen = true;
}

void serial::closeDevice( bool verbose ) {
	if( verbose ) {
		printf("%.3s  INFO    : close device /dev/ttyUSB%d\n", rPrefix.c_str( ), index );
	}
	if( close( port ) != 0 ) {
		printf( "%.3s  ERROR   : %s when closing /dev/ttyUSB%d (probably because device disconnected)\n", rPrefix.c_str( ), strerror(errno), index );
		exit( EXIT_FAILURE );
	}
	deviceOpen = false;
}

bool serial::checkCompass( ) {
	unsigned char sendVal = 0x11;
	bool retVal = false;
 	ssize_t numBytes = write( port, &sendVal, 1 );
	if( numBytes == -1 ) {
		printf("%s WARNING : cannot write command to compass (this can happen when compass just has been connected)\n", sPrefix.c_str( ));
	} else if( numBytes != 1 ) {
		printf("%s ERROR   : only one byte shall be written to compass instead of %ld\n", sPrefix.c_str( ), numBytes);
		exit( EXIT_FAILURE );
	}

	unsigned char receiveVal = 0;
	numBytes = read(port, &receiveVal, 1);
	if( numBytes > 1 ) {
		printf("%s ERROR   : only one byte shall be returned from compass or 0 for unknown device instead of %ld\n", rPrefix.c_str( ), numBytes);
		exit( EXIT_FAILURE );
	}

	if( receiveVal != 0x0f ) {
		printf("%s INFO    : not a cmps10 because because value 0x%02x instead of 0x0f\n", rPrefix.c_str( ), receiveVal );
		retVal = false;
	} else {
		printf("%s INFO    : cmps10 compass found\n", rPrefix.c_str( ) );
		retVal = true;
	}
	return retVal;
}



void serial::compassVerbose( ) {
	unsigned char sendVal = 0x12; // write value 0x12 to serial port to get 8 bit compass value
	int ii = 0;
	while( true ) {
 	 	ssize_t numBytes = write( port, &sendVal, 1 );
 		if( numBytes != 1 ) {
 			printf("%s ERROR   : only one byte shall be written to compass instead of %ld\n", sPrefix.c_str( ), numBytes);
 			exit( EXIT_FAILURE );
 		}

 		unsigned char receiveVal = 0;
		if( read(port, &receiveVal, 1) > 0 ) {
			if( (ii%100) == 0 ) {
				printf( "%s INFO    : compass value 0x%02x (%3d) = %3.0f degrees\n", rPrefix.c_str( ), receiveVal, receiveVal, 360.0*receiveVal/256.0);
			}
			ii++;
		}
	}
}

void serial::compassOpen( int index ) {
	this->index = index;
	openDevice( B9600, CS8 | CSTOPB | CREAD | CLOCAL ); // 8n2, see termios.h for more information
}

void serial::moborBoardOpen( int index ) {
	this->index = index;
	openDevice( B115200, CS8 | CREAD | CLOCAL ); // 8n1, see termios.h for more information
}

uint8_t serial::compassGetAngle( ) {
	uint8_t sendVal = 0x12; // get 8 bit angle
	int retVal = 0;
 	ssize_t numBytes = write( port, &sendVal, 1 );
 	if( numBytes == -1 ) {
		printf("%s WARNING : cannot write command to compass (this can happen when compass just has been connected)\n", sPrefix.c_str( ));
		sendError = true;
 	} else if( numBytes != 1 ) {
		printf("%s ERROR   : only one byte shall be written to compass instead of %ld\n", sPrefix.c_str( ), numBytes);
		exit( EXIT_FAILURE );
	} else {
		uint8_t angle = 0;
		numBytes = read(port, &angle, 1);
	 	if( numBytes == -1 ) {
			printf("%s WARNING : cannot read from compass (this can happen when compass just has been connected)\n", rPrefix.c_str( ));
			receiveError = true;
	 	} else if ( numBytes == 0 ) {
			printf("%s ERROR   : no data returned for compass, probably device disconnected\n", rPrefix.c_str( ));
			receiveError = true;
	 	} else if ( numBytes != 1 ) {
			printf("%s ERROR   : only one byte shall be returned from compass instead of %ld\n", rPrefix.c_str( ), numBytes);
			exit( EXIT_FAILURE );
		} else {
			retVal = (int)angle;
		}
	}
	return retVal;
}

void serial::send0BytesPayload( uint8_t command ) {
	toBoardPacketT txPacket;
	txPacket.command = command;
	txPacket.payloadSize = 0;
	sendPacket( txPacket );
	// actual return data is available in one of the next responses of the board
}

void serial::send1BytesPayload( uint8_t command, uint8_t payload ) {
	toBoardPacketT txPacket;
	txPacket.command = command;
	txPacket.pl.u8[0] = payload;
	txPacket.payloadSize = 1;
	sendPacket( txPacket );
}

void serial::send2BytesPayload( uint8_t command, uint16_t payload ) {
	toBoardPacketT txPacket;
	txPacket.command = command;
	txPacket.pl.u16[0] = payload;
	txPacket.payloadSize = 2;
	sendPacket( txPacket );
}

void serial::send2BytesPayloadXXXX( uint8_t command, uint16_t payload ) {
	toBoardPacketT tx;
	tx.command = command;
	tx.pl.u16[0] = payload;
	tx.payloadSize = 2;
	// push the new packet to the buffer so it can be send, and if needed remove oldest packet (in case of sender issue)
	if( toBoardPacket.size( ) > 255 ) { toBoardPacket.pop( ); }
    toBoardPacket.push( tx );
}

void serial::send4BytesPayload( uint8_t command, uint32_t payload ) {
	toBoardPacketT txPacket;
	txPacket.command = command;
	txPacket.pl.u32[0] = payload;
	txPacket.payloadSize = 4;
	sendPacket( txPacket );
}

void serial::sendToBoardPackets( ) {
	while( toBoardPacket.size( ) > 0 )  {
		sendPacket( toBoardPacket.front( ) );
		toBoardPacket.pop( );
	}
}

void serial::setPidProperties( uint8_t command, int p, int i, int d, int iTh ) {
	if( p < 0 || p > 65535 ) { printf("Error   : pid p out of range %d\n", p ); exit(1); }
	if( i < 0 || i > 65535 ) { printf("Error   : pid i out of range %d\n", i ); exit(1); }
	if( d < 0 || d > 65535 ) { printf("Error   : pid d out of range %d\n", d ); exit(1); }
	if( iTh < 0 || d > 65535 ) { printf("Error   : pid iTh out of range %d\n", iTh ); exit(1); }
	toBoardPacketT txPacket;
	txPacket.command = command;
	txPacket.payloadSize = 8;
	txPacket.pl.u16[0] = p;
	txPacket.pl.u16[1] = i;
	txPacket.pl.u16[2] = d;
	txPacket.pl.u16[3] = iTh;
	sendPacket( txPacket );
}


void serial::wheelBoardConfigure( ) {
	// the timeout needs to be set before the mode change, otherwise the mode will quickly switch to mode timeout
	send2BytesPayload( CMD_SET_MOTOR_TIMEOUT, 200 ); // 2.5ms * 200 = 500ms, maximal 2.5ms * 65535 = 163.8 seconds
	send0BytesPayload( CMD_GET_MOTOR_TIMEOUT );

	int timeout = 0;
	while( ( mode != MODE_DISABLE_MOTOR_POWER ) && ( timeout < 100  ) ) {
		send1BytesPayload( CMD_SET_MODE, MODE_DISABLE_MOTOR_POWER ); // prevent unexpected behavior from previous initializations
		send0BytesPayload( CMD_GET_MODE );
		timeout++;
		usleep( 40000 ); // it will take some time before the answer arrives at the receiver
	}
	if( timeout >= 10 ) { // 10*40 = 400ms
		printf("%s ERROR   : not able to change the mode to disable motor power\n", rPrefix.c_str( ) );
		sendError = true;
	} else {
		send2BytesPayload( CMD_SET_PWM_DELTA, 200 ); // max increase by the PWM per tick 648 = 100%
		send0BytesPayload( CMD_GET_PWM_DELTA );

		send2BytesPayload( CMD_SET_PWM_LIMIT, 600 ); // 648 = 100%
		send0BytesPayload( CMD_GET_PWM_LIMIT );

		// wheel motor kp > 18000 results in oscillation on free running motor
		setPidProperties( CMD_SET_PID_PRIMARY_PROPERTIES, 10000, 0000, 0, 0 ); // p, i, d, iTh
		send0BytesPayload( CMD_GET_PID_PRIMARY_PROPERTIES );

		drv8301 drv8301driver;
		send4BytesPayload( CMD_SET_DRV8301, drv8301driver.getValue( false ) ); // use false for wheels and true for ball handler
		send0BytesPayload( CMD_GET_DRV8301);

		send1BytesPayload( CMD_SET_MODE, MODE_PID_ENCODER ); // choose encoder mode for the wheel motors
		send0BytesPayload( CMD_GET_MODE );
		boardConfigured = true;
	}
}

void serial::ballHandlerBoardConfigure( ) {
	// the timeout needs to be set before the mode change, otherwise the mode will quickly switch to mode timeout
	send2BytesPayload( CMD_SET_MOTOR_TIMEOUT, 200 ); // 2.5ms * 200 = 500ms, maximal 2.5ms * 65535 = 163.8 seconds
	send0BytesPayload( CMD_GET_MOTOR_TIMEOUT );

	int timeout = 0;
	while( ( mode != MODE_DISABLE_MOTOR_POWER ) && ( timeout <= 10  ) ) { // the mode is set by the receiver thread
		send1BytesPayload( CMD_SET_MODE, MODE_DISABLE_MOTOR_POWER ); // prevent unexpected behavior from previous initializations
		send0BytesPayload( CMD_GET_MODE );
		timeout++;
		usleep( 40000 ); // it will take some time before the answer arrives at the receiver
	}
	if( timeout >= 10 ) { // 10*40 = 400ms
		printf("%s ERROR   : not able to change the mode to disable motor power\n", rPrefix.c_str( ) );
		sendError = true;
	} else {
		send2BytesPayload( CMD_SET_PWM_DELTA, 200 ); // max increase by the PWM per tick 648 = 100%
		send0BytesPayload( CMD_GET_PWM_DELTA );

		send2BytesPayload( CMD_SET_PWM_LIMIT, 600 ); // 648 = 100%
		send0BytesPayload( CMD_GET_PWM_LIMIT );

		// wheel motor kp > 18000 results in oscillation on free running motor
		setPidProperties( CMD_SET_PID_PRIMARY_PROPERTIES, 10000, 0000, 0, 0 ); // p, i, d, iTh
		send0BytesPayload( CMD_GET_PID_PRIMARY_PROPERTIES );

		setPidProperties( CMD_SET_PID_ANGLE_PROPERTIES, 4000, 0, 0, 0 ); // p, i, d, iTh, there is no feedback in the angle, so the i does not make sense
		send0BytesPayload( CMD_GET_PID_ANGLE_PROPERTIES );

		drv8301 drv8301driver;
		send4BytesPayload( CMD_SET_DRV8301, drv8301driver.getValue( false ) ); // use false for wheels and true for ball handler
		send0BytesPayload( CMD_GET_DRV8301);

		send1BytesPayload( CMD_SET_ANGLE_DIRECTION, (uint8_t) true ); // invert the angle behavior between left and right ball handler (clockwise/anti-clockwise behavior increase/decrease)
		send0BytesPayload( CMD_GET_ANGLE_DIRECTION );

		// now get the angle zero and tacho zero values which are needed to calibrate the motor for "no rotation"
		timeout = 0;
		while( ( tachoZero == 0 ) && ( timeout <= 10 ) ) { // the tachoZero value is set by the receiver thread
			usleep( 100000 ); // it will take some time before the answer arrives at the receiver
			send0BytesPayload( CMD_GET_ANGLE_TACHO_ZERO );
			timeout++;
		}
		if( timeout >= 10 ) { // 10*100 = 1000ms
			printf("%s ERROR   : not able to get the tacho zero value\n", rPrefix.c_str( ) );
			sendError = true;
		} else if( angleZero == 0 ) {
			printf("%s ERROR   : invalid angle zero value of 0\n", rPrefix.c_str( ) );
			sendError = true;
		} else {
			// configure the angle and primary set point so the motor will not spin when enabled
			send2BytesPayload( CMD_SET_PRIMARY_SETPOINT, tachoZero );
			send0BytesPayload( CMD_GET_PRIMARY_SETPOINT );

			send2BytesPayload( CMD_SET_ANGLE_SETPOINT, angleZero );
			send0BytesPayload( CMD_GET_ANGLE_SETPOINT );

			send1BytesPayload( CMD_SET_MODE, MODE_PID_ANGLE ); // choose angle mode for the ball handler motors
			send0BytesPayload( CMD_GET_MODE );

			boardConfigured = true;
			// now the board is ready to use
		}
	}
}


// reset the board by toggling the DTR pin
void serial::resetBoard( ) {
	// printf( "%s WARNING : device reset by toggling DTR\n", prefix.c_str( ) );
	// run the reset loop twice, because the DTR could already be active
	for( int ii = 0; ii < 2; ii++ )
	{
		// turn on DTR
		int iFlags = TIOCM_DTR;
		ioctl(port, TIOCMBIS, &iFlags);

		// turn off DTR
		iFlags = TIOCM_DTR;
		ioctl(port, TIOCMBIC, &iFlags);
	}
}

int serial::getFunction( int index ) {
	this->index = index;
	openDevice( B115200, CS8 | CREAD | CLOCAL ); // 8n1, see termios.h for more information

	// wait until the far end rx buffer is empty
	// so we can send a few commands without the need for flow control
	// which is not available in this single treaded board identification cycle

	// if there is nothing coming after a timeout, check if it is the compass
	farEndBufferSize = 255;
	// TODO: the next line should only be send when valid
	// printf("SER  INFO    : clear errors far end\n");
	sendClearAllErrors( ); // reset far end to prevent e.g. ack id errors when restarting Linux application, also reset pid integral value
	// TODO: use the return value of sendClearAllErrors to decide to continue instead of using the following sendWriteError
	int function = FUNCTION_UNINITIALIZED;
	farEndBufferSize = 0;
	int readTries = 0; // in case there is data coming from board, but the no space available in far end buffer size
	if( ! sendWriteError ) {
		printf("%.3s  INFO    : wait until far end receive buffer is empty ", rPrefix.c_str( ) );
		fflush( stdout );
		while( farEndBufferSize < 230 && ( ! receiveTimeOut ) && ( ! receiveReadError ) && ( readTries < 200 ) ) { // wait until far end is able to receive data (the far end receive buffer should be empty)
			// waiting to 255 takes a little longer, but we are okay with 230 bytes
			receiveBytes( true, false );
			readTries++;
			printf("." );
		}
		printf("\n");

		if( readTries >= 200 ) {
			printf( "%.3s  ERROR   : bogus return data, reset board (DTR) (this behavior needs to be tested)\n", rPrefix.c_str( ) ); exit(1);
			resetBoard( );
		}

		if( receiveReadError || readTries >= 200 ) {
			// do nothing, next cycle the read will be done again
			printf( "%.3s  WARNING : receive read error (probably device /dev/ttyUSBx just plugged in), try again\n", rPrefix.c_str( ) );
		} else if( receiveTimeOut ) {
			printf( "%.3s  INFO    : timeout occurred, check if it is a compass\n", rPrefix.c_str( ) );
			closeDevice( false );
			openDevice( B9600, CS8 | CSTOPB | CREAD | CLOCAL ); // 8n2, see termios.h for more information
			if( checkCompass( ) ) {
				function = FUNCTION_COMPASS;
			} else {
				printf( "%.3s  WARNING : unknown type of serial device, reset device by toggling DTR\n", rPrefix.c_str( ) );
				function = FUNCTION_UNKNOWN;
				resetBoard( );
			}
		} else {
			printf( "%.3s  INFO    : motor board found, reset and request board properties\n", rPrefix.c_str( ) );
			sendClearAllErrors( ); // reset far end to prevent e.g. ack id errors when restarting linux application, also reset pid integral value
			// TODO: use the return value of the sendClearAllErrors to decide to bail out
			send0BytesPayload( CMD_AA_GET_BOARD );

			// get application ID
			printf( "%.3s  INFO    : wait for response packet from board ", rPrefix.c_str( ) );
			readTries = 0;
			while( rxPacket.responseType != RESP_AA_BOARD  ) {
				receiveBytes( true, false );
				printf(".");
				if( readTries % 100 == 0 ) {
					send0BytesPayload( CMD_AA_GET_BOARD );
				}
				usleep( 10 );
				readTries++;
			}
			printf("\n");

			if( rxPacket.payloadSize != 24 ) {
				printf( "%.3s  ERROR   : invalid payload size for board response %d\n", rPrefix.c_str( ), rxPacket.payloadSize ); exit(1);
			} else {
				printf("%.3s  INFO    : Application ID 0x%04x (%d), Software Version : 0x%04x (%d)\n", rPrefix.c_str( ), rxPacket.pl.u16[2], rxPacket.pl.u16[2], rxPacket.pl.u16[4], rxPacket.pl.u16[4] );
				if( rxPacket.pl.u16[3] != HARDWARE_VERSION ) {
					printf( "%.3s  ERROR   : invalid different hardware version between linux 0x%04x (%d) and board 0x%04x (%d), abort\n", rPrefix.c_str( ), HARDWARE_VERSION, HARDWARE_VERSION, rxPacket.pl.u16[3], rxPacket.pl.u16[3] );
		 			exit( EXIT_FAILURE );
				}
				if( rxPacket.pl.u16[4] != SOFTWARE_VERSION ) {
					printf( "%.3s  ERROR   : invalid different software version between linux 0x%04x (%d) and board 0x%04x (%d), abort\n", rPrefix.c_str( ), SOFTWARE_VERSION, SOFTWARE_VERSION, rxPacket.pl.u16[4], rxPacket.pl.u16[4] );
		 			exit( EXIT_FAILURE );
				}
				if( rxPacket.pl.u16[2] == 0 ) {
					printf( "%3.s  ERROR   : invalid Application ID %d, probably short, abort\n", rPrefix.c_str( ), rxPacket.pl.u16[2] );
		 			exit( EXIT_FAILURE );
				} else if( rxPacket.pl.u16[2] == 1 ) {
					function = FUNCTION_BALL_RIGHT;
				} else if( rxPacket.pl.u16[2] == 2 ) {
					function = FUNCTION_WHEEL_RIGHT;
				} else if( rxPacket.pl.u16[2] == 3 ) {
					function = FUNCTION_WHEEL_REAR;
				} else if( rxPacket.pl.u16[2] == 4 ) {
					function = FUNCTION_WHEEL_LEFT;
				} else if( rxPacket.pl.u16[2] == 5 || rxPacket.pl.u16[2] == 6 ) { // TODO: remove 6 for ball handler left !
					function = FUNCTION_BALL_LEFT;
				} else if( rxPacket.pl.u16[2] == 7 ) {
					printf( "%.3s  ERROR   : invalid Application ID %d, probably open, abort\n", rPrefix.c_str( ), rxPacket.pl.u16[2] );
		 			exit( EXIT_FAILURE );
				}
			}
		}
	}
	return function;
}
