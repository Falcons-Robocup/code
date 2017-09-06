 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cCompass.cpp
 *
 *  Created on: Apr 27, 2014
 *      Author: Tim Kouters
 */

#include "int/compass/cCompass.hpp"
#include "int/compass/CMPS10_commands.h"

#include <fcntl.h>
#include <stdexcept>
#include <iostream>
#include <math.h>

#include "int/config/cCompassConfig.hpp"

using std::string;
using std::size_t;
using std::runtime_error;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

cCompass::cCompass(std::string &ttyUSB)
{
//	TRACE(">");
	//string deviceName;
	//cCompassConfig::getInstance().getDeviceName(deviceName);

	_deviceHandle = open(ttyUSB.c_str(), O_RDWR | O_NOCTTY);

	if (_deviceHandle == -1)
	{
		throw runtime_error("Cannot open device");
	}

	/* Open communication */
	openPort();
//	TRACE("<");
}

cCompass::~cCompass()
{
//	TRACE(">");
	if (_deviceHandle != -1)
	{
	    /* Close communication */
	    closePort();

	    /* Close device handle */
		close(_deviceHandle);
	}
//	TRACE("<");
}

void cCompass::getCompassAngle(float &angle)
/*
 * out: float with relative angle from own goal with range 0..360 degrees
 */
{
//	TRACE(">");
	try
	{
		readRawCompassAngle(_currentRawAngle);
		angle = fabs(fmod(720.0 - _currentRawAngle + cCompassConfig::getInstance().getHomeGoalAngle(), 360.0));

	} catch (exception &e)
	{
		throw e;
	}
//	TRACE("<");
}

void cCompass::getRawCompassAngle(float &angle)
/*
 * out: float with absolute angle from own goal with range 0..360 degrees
 */
{
//	TRACE(">");
	try
	{
		readRawCompassAngle(_currentRawAngle);
		angle = _currentRawAngle;

	} catch (exception &e)
	{
		throw e;
	}
//	TRACE("<");
}

void cCompass::readRawCompassAngle(float &angle)
/*
 * out: float with absolute angle of the compass with range 0..360
 */
{
//	TRACE(">");
	int term_ret = 0;
	unsigned int uangle = 0;
	unsigned int high_byte, low_byte = 0;
    unsigned char char_array[2] = { '\0', };

	try
	{
		while (term_ret != 0)
		{
			term_ret = tcgetattr(_deviceHandle, &_termiosStr);
		}

		/* fetch char array with data */
		sendCommand(CMPS10::GET_ANGLE_16BIT);
		readData(2, char_array);

		/* Calculate angle */
		high_byte = char_array[0];
		low_byte = char_array[1];
		uangle = ((high_byte << 8) + low_byte);

		/* angle integer has a value range of 0..3600 */
		angle = ((float)uangle) / 10.0;

	} catch (exception &e)
	{
		throw e;
	}
//	TRACE("<");
}

void cCompass::openPort()
{
//	TRACE(">");
	_termiosStr.c_iflag = IGNBRK;
	_termiosStr.c_iflag &= ~(IXON|IXOFF|IXANY);
	_termiosStr.c_oflag = 0;
	_termiosStr.c_lflag = 0;
	_termiosStr.c_cflag |= (CLOCAL | CREAD);         // Enable the receiver and set local mode
	_termiosStr.c_cflag &= ~(PARENB | PARODD);                 // No parity bit
	_termiosStr.c_cflag |= CSTOPB;                 // Set 2 stop bits
	_termiosStr.c_cflag &= ~CRTSCTS; // hardware control off
	_termiosStr.c_cflag &= ~CSIZE;                  // Set the character size
	_termiosStr.c_cflag |= CS8;
	_termiosStr.c_cc[VMIN] = 60;
	_termiosStr.c_cc[VTIME] = 1;

    if(cfsetispeed(&_termiosStr, B9600) < 0)
    {
        throw runtime_error("Failed to set input baud-rate");
    }

    if (cfsetospeed(&_termiosStr, B9600) < 0)
    {
        throw runtime_error("Failed to set output baud-rate");
    }

	if(tcsetattr(_deviceHandle, TCSANOW, &_termiosStr) < 0)
	{
	    throw runtime_error("Failed to set new tty settings");
	}

	if(tcflush(_deviceHandle, TCIOFLUSH))
	{
	    throw runtime_error("Failed to flush");
	}
//	TRACE("<");
}

void cCompass::closePort()
{
//	TRACE(">");
	int term_ret = -1;

	while (term_ret != 0)
	{
		term_ret = tcsetattr(_deviceHandle, TCSANOW, &_termiosStr);
	}
//	TRACE("<");
}

void cCompass::readData(size_t numberOfChars, unsigned char *charArray)
{
//	TRACE(">");
    size_t bytes_read = 0;
	try
	{
	    bytes_read = read(_deviceHandle, charArray, numberOfChars);
		if(bytes_read != numberOfChars)
		{
		    throw runtime_error("Invalid number of bytes read");
		}
	} catch (exception &e)
	{
		delete charArray;
		charArray = NULL;
		throw e;
	}
//	TRACE("<");
}

void cCompass::sendCommand(char command)
{
//	TRACE(">");
	int bytes_written = 0;

	try
	{
		bytes_written = write(_deviceHandle, &command, sizeof(command));

		if (bytes_written != sizeof(command))
		{
			throw runtime_error("Invalid number of bytes written");
		}
	} catch (exception &e)
	{
		throw e;
	}
//	TRACE("<");
}

void cCompass::getVersion(unsigned char &version)
{
//	TRACE(">");
    int term_ret = 0;
    unsigned char char_array[1] = { '\0', };

    try
    {
        while (term_ret != 0)
        {
            term_ret = tcgetattr(_deviceHandle, &_termiosStr);
        }

        /* fetch char array with data */
        sendCommand(CMPS10::GET_VERSION);
        readData(1, char_array);

        version = char_array[0];

    } catch (exception &e)
    {
        throw e;
    }
//    TRACE("<");
}
