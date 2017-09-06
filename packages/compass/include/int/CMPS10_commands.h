 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CMPS10_commands.h
 *
 *  Created on: May 01, 2014
 *      Author: Tim Kouters
 *      Source: http://www.robot-electronics.co.uk/htm/cmps10ser.htm
 */

#ifndef CMPS10_COMMANDS_H_
#define CMPS10_COMMANDS_H_

namespace CMPS10
{

const static char OK_COMMAND = 0x55;
const static char GET_VERSION = 0x11;
const static char GET_ANGLE_8BIT = 0x12;
const static char GET_ANGLE_16BIT = 0x13;
const static char GET_PITCH = 0x14;
const static char GET_ROLL = 0x15;
const static char GET_MAG_RAW = 0x21;
const static char GET_ACCEL_RAW = 0x22;
const static char GET_ALL = 0x23;
const static char CALIBRATE_EN1 = 0x31;
const static char CALIBRATE_EN2 = 0x45;
const static char CALIBRATE_EN3 = 0x5A;
const static char CALIBRATE = 0x5E;
const static char RESTORE1 = 0x6A;
const static char RESTORE2 = 0x7C;
const static char RESTORE3 = 0x81;
const static char BAUD_19200 = 0xA0;
const static char BAUD_38400 = 0xA1;
}

#endif /* CMPS10_COMMANDS_H_ */
