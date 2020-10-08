 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include "drv8301.hpp"

using namespace std;


drv8301::drv8301(  ) {
}

// initialize all values
uint32_t drv8301::getValue( bool ballHandler ) {
	// setDrv8301( 0x288, (0x00 | (3<<2)) ); // 80V/V
	// setDrv8301( (0x288 | (1<<2) ), 0x00 ); // reset gate driver
	// setDrv8301( 0x288, 0x00 );
	uint16_t control1 = 0;
	control1 |=  0<<0; // GATE_CURRENT : Gate drive peak current 1.7 A
	// control1 |=  1<<0; // GATE_CURRENT : Gate drive peak current 0.7 A
	control1 |=  1<<2; // GATE_RESET : Reset gate driver latched faults (reverts to 0) (required if you got the drv8301 error shutdown occurred (FAULT))
	control1 |=  1<<3; // PWM_MODE : 3 PWM inputs
	control1 |=  0<<4; // OCP_MODE : Current limit (works together with OC_ADJ_SET)
	// control1 |=  1<<4; // OCP_MODE : OC latch shut down (requires a gate reset to resolve)
	// OC_ADJ_SET VDS = iOverCurrent x rdsOnFet (irf7749) => 60mV / 1.1mOhm = 54.5A => does not work, gives quickly errors when changing direction
	// use current limit instead of shutdown
	// control1 |= 0<<6; // OC_ADJ_SET : Over current Adjustment = 60mV
	if( ballHandler ) {
		// TODO: determine lower value for over current for ball handler
		control1 |= 10<<6; // OC_ADJ_SET : Over current Adjustment = 197mV
	} else {
		control1 |= 10<<6; // OC_ADJ_SET : Over current Adjustment = 197mV
	}
	// control1 |= 31<<6; // OC_ADJ_SET : Over current Adjustment = 2.4V
	uint16_t control2 = 0;
	control2 |= 0<<0; // OCTW_MODE : Report both over temperature (OT) and over current (OC) at nOCTW pin
	control2 |= 2<<2; // GAIN : Gain of shunt amplifier: 40 V/V, with 30A ADC range between 0.05V to 2.45V, so best tradeoff between high current and noise
	// control2 |= 1<<4; // DC_CAL_CH1 : calibrate shunt amplifier 1
	// control2 |= 1<<5; // DC_CAL_CH2 : calibrate shunt amplifier 2
	control2 |= 0<<6; // OC_TOFF : Cycle by cycle = the MOSFET on which over current has been detected on will shut off until the next PWM cycle

	return( control2<<16 | control1 );
}



