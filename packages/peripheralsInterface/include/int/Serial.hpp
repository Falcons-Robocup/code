 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
