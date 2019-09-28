 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <queue>
#include <mutex>
#include <termios.h>
#include <unistd.h>

#include "motorBoard.hpp"
#include "drv8301.hpp"

#ifndef SERIAL_HPP
#define SERIAL_HPP

typedef enum functionList {
	FUNCTION__AA_DO_NOT_USE_ZERO = 0,
	FUNCTION_BALL_LEFT,
	FUNCTION_BALL_RIGHT,
	FUNCTION_COMPASS,
	FUNCTION_UNINITIALIZED,
	FUNCTION_UNKNOWN,
	FUNCTION_WHEEL_LEFT,
	FUNCTION_WHEEL_REAR,
	FUNCTION_WHEEL_RIGHT,
	FUNCTION_ZZ_DO_NOT_USE_LAST
} functionListT;

class serial {
	
	#define SOF 0x5a

private:
	int port; // serial port number used for read and write to the serial port
	int index; // /dev/ttyUSB index
	std::string sPrefix; // used as prefix during printing for send
	std::string rPrefix; // used as prefix during printing for receive
	std::string name; // used as name during printing

	uint8_t txPacketAckId = 0; // first packet start with ackId 0, then increment (wrap around) for each following packet

	fromBoardPacketT rxPacket, rxPacketTmp; // use send packet definition from board as receive packet for pc
	
	motorBoard motor;

	// used in receivedBytes
	int state; // state 0 = search SOF, state 1 = complete the packet
	int rxBytes; // current packet byte size
	char *rxPacketTmpPointer; // for efficient iterating through the rx packet
	size_t receivedPacketsCnt;
	size_t receivedPacketsCntPrev; // used to keep track if for a while no packet was received
	size_t receivedPacketsCntPrevSend; // used to keep track if for a while no packet was received
	size_t receiveChecksumFail;
	size_t receiveAckIdMis;
	bool receiveTimeOut;
	bool receiveReadError;
	bool sendWriteError;
	bool sendStopped;

	int farEndBufferSize;
	size_t sendPacketsCnt;
	size_t sofAlignmentCounter;
	int16_t primarySetPoint; // store setPoint sent to board for printing during receive to compare with actual value

	int farEndAckIdMis;
	int farEndChecksumFail;
	int farEndPacketOverwritten;
	int farEndBufferOverflow;
	int farEndWrongPayloadSize;

	uint16_t applicationId;

	uint8_t mode; // store mode sent to board to select the correct printing during receive
	uint16_t angleZero;
	uint16_t tachoZero;
	bool boardConfigured;

	uint8_t ackId;
	int initialized;
	
	bool receiveError;
	bool sendError;
	size_t deviceErrorThreshold;
	size_t deviceErrorPrintCount;
	bool deviceOpen; // handshake for the receive thread to inform the send thread the device open


	std::mutex deviceErrorMutex;

	void sendPacket( toBoardPacketT tx );
	void processReceivedPacket( );
	void sendClearAllErrors( );

	int clearRemoteErrors( );
	bool rxPacketAckIdCheck( );
	bool rxPacketChecksumCheck( );

	bool checkCompass( );
	void compassVerbose( );
	void openDevice( speed_t speed, tcflag_t cflag );

public:
	serial( );
	~serial( );
	void init( std::string rPrefix, std::string sPrefix, std::string name, bool verbose );

	void resetBoard( );
	void compassOpen( int index );
	void moborBoardOpen( int index );
	void sendClearKnownErrors( fromBoardPacketT rxPacket );

	uint8_t compassGetAngle( );

	void closeDevice( bool verbose );
	bool getDeviceOpen( ) { return deviceOpen; }

	void providePacket( toBoardPacketT packet );
	int receiveBytes( bool init, bool verbose );
	size_t getReceivedPacketsCnt( ) { return receivedPacketsCnt; }

	int getFunction( int index );
	uint8_t getMode( ) { return mode; }

	void send0BytesPayload( uint8_t command );
	void send1BytesPayload( uint8_t command, uint8_t payload );
	void send2BytesPayload( uint8_t command, uint16_t payload );
	void send2BytesPayloadXXXX( uint8_t command, uint16_t payload );

	void send4BytesPayload( uint8_t command, uint32_t payload );
	void sendToBoardPackets( );
	void setPidProperties( uint8_t command, int p, int i, int d, int iTh );

	uint16_t getAngleZero( ) { return angleZero; }
	uint16_t getTachoZero( ) { return tachoZero; }
	bool getBoardConfigured( ) { return boardConfigured; }
	void setBoardConfigured( bool value ) { boardConfigured = value; }
	void wheelBoardConfigure( );
	void ballHandlerBoardConfigure( );

	int getPort( ) { return port; }
	int getFarEndBufferSize( ) { return farEndBufferSize; }

	void checkReceive( );
	bool getFailing( ) { return ( receiveError || sendError ); }
//	void setDeviceError( bool value ) { deviceError = value; }
	void setSendStopped( ) { sendStopped = true; }
	bool getSendStopped( ) { return sendStopped; }

	std::queue<fromBoardPacketT> fromBoardPacket;
	std::queue<toBoardPacketT> toBoardPacket;


	void printSendPacket( toBoardPacketT txPacket ) {
		motor.printSendPacket( sPrefix, txPacket );
	}
	void printReceivedPacket( fromBoardPacketT rxPacket ) {
		motor.printReceivedPacket( rPrefix, rxPacket, mode, primarySetPoint, applicationId );
	}
	void printRemoteErrors( fromBoardPacketT rxPacket ) {
		motor.printRemoteErrors( rPrefix, rxPacket );
	}
};

#endif
