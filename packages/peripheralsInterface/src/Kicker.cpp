 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Kicker.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Edwin Schreuder
 */

#include <FalconsCommon.h>

#include "int/Kicker.hpp"

const float KickerMaxHeight = 205;
const float KickerDefaultShootPower = 50;
const float KickerDefaultHeight = 100;
const float KickerDefaultLeverSpeed = 50;

Kicker::Kicker(IoBoard &ioBoard) :
		ioBoard(ioBoard) {
	height = KickerDefaultHeight;
	shootPower = KickerDefaultShootPower;
	leverSpeed = KickerDefaultLeverSpeed;
}

Kicker::~Kicker() {

}

void Kicker::setShootPower(float _shootPower) {
	shootPower = _shootPower;
}

float Kicker::getShootPower() {
	return shootPower;
}

void Kicker::setHeight(float _height) {

	if (_height > KickerMaxHeight) {
		height = KickerMaxHeight;
	}
	else if (_height < 0) {
		height = 0;
	}
	else {
		height = _height;
	}
}

float Kicker::getHeight() {
	return height;
}

void Kicker::setLeverSpeed(float speed) {
	leverSpeed = speed;
	ioBoard.setLeverSpeed(((unsigned char) leverSpeed) + 100);
}

float Kicker::getLeverSpeed() {
	return leverSpeed;
}

void Kicker::move() {
	ioBoard.setHeight((unsigned char) height);
}

void Kicker::shoot() {
	ioBoard.setShoot((unsigned char) shootPower);
}

void Kicker::home() {
	ioBoard.setHome();
}
