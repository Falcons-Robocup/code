 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Serial.cpp
 *
 *  Created on: Jan 1, 2016
 *      Author: Prabhu Mani, Edwin Schreuder
 */

#include <exception>
#include <iostream>

#include <cstring>
#include <cerrno>

#include <cmath>
#include <fcntl.h>
#include <int/PeripheralsInterfaceExceptions.hpp>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "int/Serial.hpp"

using namespace std;

Serial::Serial(string portName, speed_t baudrate, float timeout, unsigned char minCharacters) :
		portName(portName), baudrate(baudrate), timeout(timeout), portHandle(-1), minCharacters(minCharacters) {
	openPort();
}

Serial::~Serial() {
	closePort();
}

void Serial::openPort() {

	portHandle = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (portHandle == -1) {
		throw serialException::PortOpenFailed{portName};
	}

	int result = fcntl(portHandle, F_SETFL, O_RDWR);
	if (result < 0) {
		throw serialException::PortOpenFailed{portName};
	}

	setSettings();

	flushPort();
}

void Serial::setSettings() {
	struct termios tio = {0,};
	tio.c_ospeed = 0;
	tio.c_iflag = 0;	
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
	tio.c_lflag = 0;
	if (timeout > 0) {
		tio.c_cc[VMIN] = 0; // Minimum number of characters for noncanonical read (MIN).
		tio.c_cc[VTIME] = (unsigned char) round(timeout * 10); // Timeout in deciseconds for noncanonical read (TIME) 5 = 500ms //
	}
	else {
		tio.c_cc[VMIN] = minCharacters;
		tio.c_cc[VTIME] = 0;
	}
	tio.c_ispeed = 0;

	// do not use O_NONBLOCK, because we want to wait for a reply until the VTIME expires
	cfsetospeed(&tio, baudrate);
	cfsetispeed(&tio, baudrate);
	tcsetattr(portHandle, TCSANOW, &tio);
}

void Serial::closePort() {
	if (portHandle >= 0) {
		int result = close(portHandle);
		if (result) {
			cerr << "ERROR   : Could not close port: " << strerror(errno) << endl;
		}
	}
	portHandle = -1;
}

void Serial::flushPort(){
	int result = tcflush(portHandle, TCIOFLUSH);
	if (result) {
		throw(runtime_error("Could not flush port: " + string(strerror(errno))));
	}
}

void Serial::writePort(unsigned char * buffer, size_t length) {
	// Write a number number of bytes to the serial port.
	 int numBytes = write(portHandle, (void *) buffer, length);
	if (((size_t) numBytes) != length) {
		throw serialException::WriteBytesError(strerror(errno), portName);
	}
}

void Serial::writePort(const vector<unsigned char> &buffer) {
	// Write a number number of bytes to the serial port.
	 int numBytes = write(portHandle, (void *) buffer.data(), buffer.size());
	if (((size_t) numBytes) != buffer.size()) {
		throw serialException::WriteBytesError(strerror(errno), portName);
	}
}

int Serial::readPort(unsigned char * buffer, size_t packetSize) {
	// Get a number of bytes from the serial port.
	int numBytes = read(portHandle, buffer, packetSize);
	if (numBytes == -1) {
		throw serialException::ReadBytesError(strerror(errno), portName);
	}

	return numBytes;
}

vector<unsigned char> Serial::readPort(size_t packetSize) {
	// Get a number of bytes from the serial port.
	vector<unsigned char> buffer(packetSize);

	int numBytes = read(portHandle, buffer.data(), packetSize);
	if (numBytes == -1)
	{
		throw serialException::ReadBytesError(strerror(errno), portName);
	}
	buffer.resize(numBytes);

	return buffer;
}


int Serial::getFileDescriptor() {
	return portHandle;
}

speed_t Serial::getBaudrate() {
	return baudrate;
}

void Serial::setBaudrate(speed_t value) {
	baudrate = value;

	setSettings();
}

unsigned char Serial::getMinCharacters() {
	return minCharacters;
}

void Serial::setMinCharacters(unsigned char _minCharacters) {
	minCharacters = _minCharacters;

	setSettings();
}

string Serial::getPortName(){
	return portName;
}

void Serial::setDTR() {
	// Turn on DTR
	int flag = TIOCM_DTR;
	ioctl(portHandle, TIOCMBIS, &flag);
}

void Serial::unsetDTR() {
	// Turn off DTR
	int flag = TIOCM_DTR;
	ioctl(portHandle, TIOCMBIC, &flag);
}

void Serial::setRTS() {
	// Turn on RTS
	int flag = TIOCM_RTS;
	ioctl(portHandle, TIOCMBIS, &flag);
}

void Serial::unsetRTS() {
	// Turn off DTR
	int flag = TIOCM_RTS;
	ioctl(portHandle, TIOCMBIC, &flag);
}
