 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * MotionBoard.hpp
 *
 *  Created on: May 3, 2016
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_MOTORS_MOTIONBOARD_HPP_
#define INCLUDE_INT_MOTORS_MOTIONBOARD_HPP_

#include <string>

#include "int/motors/MotorControllerBoard.hpp"

#include "int/motors/Communication.hpp"
#include "int/motors/DeviceManager.hpp"

using namespace std;

// The encoder used gives 512 pules per revolution. Quadrature encoding is used.
// Furthermore, the encoder is located on a gear with ratio 1:12. The minimal
// rotation the encoder can therefore detect is
// theta_min = (2*pi)/(4*12*512) [rad] = approx 0.00025566346 [rad]
// The robot wheel has a circumference of 0.317 [m]. The distance traveled can
// be calculated by l = theta * r, r can be obtained from the circumference,
// r = C / (2*pi). Therefore
//
// r = C / (2*pi)
// L = theta_min * r
//
// => L = ((2*pi) / (4*12*512)) * (0.317 / 2*pi) = 0.317 / (4*12*512)
//
// Is the minimum distance traveled which can be detected.
static const double oneWheelTick = 0.317 / (4.0 * 12.0 * 512.0);

// TODO: THIS IS UTTERLY WRONG!! We have a 400 Hz sample frequency, not 500.
// Therefore the reported value has 20% delta.
// 1 tick = 0.00001289876 meters per 2ms
// max ticks = 1000 = 0.01289876 meters per 2ms
// 0.01289876 meters per 2ms * 400 = 5.159504 m/s
// 5.159504 m/s == 1000 ticks
// 1000 / 5.159504 = 193.817080091 ticks for 1 m/s
// EKPC : We measure for 2ms every 2.5ms
// Which means we get the ticks originating for 2ms, but actual time passed is 2.5ms
// This means we should use 800 as max ticks.
static const float velocityToEncoderValue = 193.817080091;

// The sample frequency is 400 Hz in the motor boards. The velocity represented
// by the motor board is the difference of ticks between two adjacent samples.
// Therefore, to calculate velocity of the motor, one has to divide the velocity
// in ticks per sample by the sample time (or multiply with the sample frequency)
// and multiply by the amount of distance per tick.
static const double newEncoderToVelocityFactor = oneWheelTick * 400.0;
static const double newVelocityToEncoderFactor = 1.0/newEncoderToVelocityFactor;

// PWM min/max = (-)318*256
// Encoder ticks min/max = (-)1000
// Velocity min/max = (-)6.44938m/s

static constexpr float oneTimeTick = 0.002; // velocity is measured very 2ms

enum MotionBoardType {
	MOTION_BOARD_RIGHT,
	MOTION_BOARD_LEFT,
	MOTION_BOARD_REAR
};

class MotionBoard : public MotorControllerBoard {
public:
	MotionBoard(MotionBoardType type, DeviceManager &deviceManager);

	MotionBoardDataOutput getBoardData();
	void setSetpoint(float setpoint);
	void setSettings(MotionBoardSettings settings);

protected:
	MotionBoardData motionData;
	MotionBoardSettings settings;

	virtual void update();
	virtual void configure();
	virtual bool isConfigurationDone();

	virtual void handleDefaultResponse(ReceivePackage &package);
};

#endif /* INCLUDE_INT_MOTORS_MOTIONBOARD_HPP_ */
