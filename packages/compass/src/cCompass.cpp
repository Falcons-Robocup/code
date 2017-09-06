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

#include "int/cCompass.h"
#include "int/CMPS10_commands.h"

#include <fcntl.h>
#include <stdexcept>
#include <iostream>
#include <math.h>

namespace Compass
{

using std::string;
using std::size_t;
using std::runtime_error;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

cCompass::cCompass(string device_name, unsigned baud_rate, int homegoal_angle,
		bool debug_enabled = false)
{
	device_handle = open(device_name.c_str(), O_RDWR | O_NOCTTY);

	if (device_handle == -1)
	{
		throw runtime_error("Cannot open device");
	}

	/* Open communication */
	open_port();

	if (baud_rate != 9600)
	{
	    /* Set serial link to given baud-rate */
	    set_baudrate(baud_rate);
	}

	this->baud_rate = baud_rate;
	this->debug_enabled = debug_enabled;
	this->homegoal_angle = homegoal_angle;

	/* Start timer */
	tmrReadCompass = hROS.createTimer(ros::Duration(0.1), &cCompass::cbReadCompass, this);
}

cCompass::~cCompass()
{
	if (device_handle != -1)
	{
	    /* Close communication */
	    close_port();

	    /* Close device handle */
		close(device_handle);
	}
}

void cCompass::get_compass_angle(float *angle_p)
/*
 * out: float with relative angle from own goal with range 0..360 degrees
 */
{
	try
	{
		*angle_p = fabs(fmod(360.0 + current_raw_angle - homegoal_angle, 360.0));

	} catch (exception &e)
	{
		throw e;
	}
}

void cCompass::get_raw_compass_angle(float *angle_p)
/*
 * out: float with absolute angle from own goal with range 0..360 degrees
 */
{
	try
	{
		*angle_p = current_raw_angle;

	} catch (exception &e)
	{
		throw e;
	}
}

void cCompass::read_raw_compass_angle(float *angle_p)
/*
 * out: float with absolute angle of the compass with range 0..360
 */
{
	int term_ret = 0;
	unsigned int angle = 0;
	unsigned int high_byte, low_byte = 0;
    unsigned char char_array[2] = { '\0', };

	try
	{
		/* Verify that pointer are not NULL */
		if (angle_p == NULL)
		{
			throw runtime_error("invalid angle pointer");
		}

		while (term_ret != 0)
		{
			term_ret = tcgetattr(device_handle, &termios_str);
		}

		/* fetch char array with data */
		send_command(CMPS10::GET_ANGLE_16BIT);
		read_data(2, char_array);

		/* Calculate angle */
		high_byte = char_array[0];
		low_byte = char_array[1];
		angle = ((high_byte << 8) + low_byte);

		if (debug_enabled)
		{
			cout << "the angle is " << angle << endl;
		}

		/* angle integer has a value range of 0..3600 */
		*angle_p = ((float)angle) / 10.0;

	} catch (exception &e)
	{
		throw e;
	}
}

void cCompass::open_port()
{
    termios_str.c_iflag = IGNBRK;
    termios_str.c_iflag &= ~(IXON|IXOFF|IXANY);
    termios_str.c_oflag = 0;
    termios_str.c_lflag = 0;
    termios_str.c_cflag |= (CLOCAL | CREAD);         // Enable the receiver and set local mode
    termios_str.c_cflag &= ~(PARENB | PARODD);                 // No parity bit
    termios_str.c_cflag |= CSTOPB;                 // Set 2 stop bits
    termios_str.c_cflag &= ~CRTSCTS; // hardware control off
    termios_str.c_cflag &= ~CSIZE;                  // Set the character size
    termios_str.c_cflag |= CS8;
    termios_str.c_cc[VMIN] = 60;
    termios_str.c_cc[VTIME] = 1;

    if(cfsetispeed(&termios_str, B9600) < 0)
    {
        throw runtime_error("Failed to set input baud-rate");
    }

    if (cfsetospeed(&termios_str, B9600) < 0)
    {
        throw runtime_error("Failed to set output baud-rate");
    }

	if(tcsetattr(device_handle, TCSANOW, &termios_str) < 0)
	{
	    throw runtime_error("Failed to set new tty settings");
	}

	if(tcflush(device_handle, TCIOFLUSH))
	{
	    throw runtime_error("Failed to flush");
	}
}

void cCompass::close_port()
{
	int term_ret = -1;

	while (term_ret != 0)
	{
		term_ret = tcsetattr(device_handle, TCSANOW, &termios_str);
	}
}

void cCompass::read_data(size_t number_of_chars, unsigned char *char_array)
{
    size_t bytes_read = 0;
	try
	{
	    bytes_read = read(device_handle, char_array, number_of_chars);
		if(bytes_read != number_of_chars)
		{
		    throw runtime_error("Invalid number of bytes read");
		}
	} catch (exception &e)
	{
		delete char_array;
		char_array = NULL;
		throw e;
	}
}

void cCompass::send_command(char command)
{
	int bytes_written = 0;

	try
	{
		bytes_written = write(device_handle, &command, sizeof(command));

		if (bytes_written != sizeof(command))
		{
			throw runtime_error("Invalid number of bytes written");
		}
	} catch (exception &e)
	{
		throw e;
	}
}

void cCompass::set_baudrate(int rate)
{
    int brate = 0;
    unsigned char bcommand = '\0';
    unsigned char ok_command = '\0';

    switch(rate)
    {
        case 19200:
        {
            brate = B19200;
            bcommand = CMPS10::BAUD_19200;
            break;
        }

        case 38400:
        {
            bcommand = CMPS10::BAUD_38400;
            brate = B38400;
            break;
        }

        default:
        {
            cerr << "Unsupported baud-rate: " << rate << endl;
            throw runtime_error("Unsupported baud-rate");
        }
    }

    send_command(bcommand);

    if(cfsetispeed(&termios_str, brate) < 0)
    {
        throw runtime_error("Failed to set input baud-rate");
    }

    if (cfsetospeed(&termios_str, brate) < 0)
    {
        throw runtime_error("Failed to set output baud-rate");
    }

    read_data(1, &ok_command);

    if (ok_command != CMPS10::OK_COMMAND)
    {
        throw runtime_error("Failed to retrieve OK command");
    }
}

void cCompass::cbReadCompass(const ros::TimerEvent& e)
{
	float angle = 0.0;

	try {
		/* Fetch raw compass data */
		read_raw_compass_angle(&angle);
		current_raw_angle = angle;

	} catch (exception &e) {
		throw e;
	}
}

} /* namespace Compass */
