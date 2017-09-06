 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cCompass.h
 *
 *  Created on: Apr 27, 2014
 *      Author: Tim Kouters
 */

#ifndef CCOMPASS_H_
#define CCOMPASS_H_

#include <stdio.h>
#include <string>
#include <termios.h>

#include <ros/timer.h>
#include <ros/node_handle.h>

namespace Compass {

class cCompass {
public:
	//cCompass(); No default constructor, device_name is mandatory
	cCompass(std::string device_name, unsigned int baud_rate, int homegoal_angle, bool debug_enabled);
	virtual ~cCompass();
	void get_compass_angle(float *angle_p);
	void get_raw_compass_angle(float *angle_p);

private:
	bool debug_enabled;
	int  device_handle;
	unsigned int baud_rate;
	int homegoal_angle;
	float current_raw_angle;
	termios termios_str;
	ros::Timer tmrReadCompass;
	ros::NodeHandle hROS;

	void open_port();
	void close_port();
	void read_data(std::size_t number_of_chars, unsigned char *char_array);
	void send_command(char command);
	void set_baudrate(int rate);
	void read_raw_compass_angle(float *angle_p);

	void cbReadCompass(const ros::TimerEvent& e);

};

} /* namespace Compass */
#endif /* CCOMPASS_H_ */
