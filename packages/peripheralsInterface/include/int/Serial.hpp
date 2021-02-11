// Copyright 2016-2018 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Serial.hpp
 *
 *  Created on: Jan 1, 2016
 *      Author: Prabhu Mani
 */

#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <string>
#include <vector>

#include <termios.h>
#include <unistd.h>

using namespace std; // TODO: this is not a nice thing to do in header files .... it affects other headers

class Serial {

public:
	Serial(string portName, speed_t baudrate, float timeout, unsigned char minCharacters);
	~Serial();

	int getFileDescriptor();

	string getPortName();

	speed_t getBaudrate();
	void setBaudrate(speed_t baudrate);

	unsigned char getMinCharacters();
	void setMinCharacters(unsigned char minCharacters);

	// Send functions
	void writePort(unsigned char * buffer, size_t length);
	void writePort(const vector<unsigned char> &buffer);

	// Receive functions
	int readPort(unsigned char * buffer, size_t packetSize);
	vector<unsigned char> readPort(size_t packetSize);
	void flushPort();

	// Pin manipulation functions:
	void setDTR();
	void unsetDTR();
	void setRTS();
	void unsetRTS();

private:
	void openPort();
	void closePort();

	void setSettings();

	string portName;
	speed_t baudrate;
	float timeout;
	int portHandle;
	size_t minCharacters;
};
#endif /* SERIAL_HPP_ */
