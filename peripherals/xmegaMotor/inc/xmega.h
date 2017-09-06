// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_XMEGA_H
#define INCLUDED_XMEGA_H

#include <avr/io.h>
#include <stdbool.h>

#include "global.h"


// Atmel Xmega specific functions

// pin definitions on motor boards from schematic kd1301002 revision 1.1

// pa0 vref2.5
// pa1 gnd
// pa2 so1 from drv8301
// pa3 so2 from drv8301
// pa4 vpwr sense 2.8V = 24v power input
// pa5 ntc1
// pa6 ntc2
// pa7 tacho or ntc3

// spi interface on port c defined in spiMaster.c

// pwm a to drv8301
#define PWM_A IOPORT_CREATE_PIN(PORTD, 2)
// pwm b to drv8301
#define PWM_B IOPORT_CREATE_PIN(PORTD, 3)

// xtal2 - xtal1 16MHz



/*! \brief configure the used ports
 */
void initXmega();

/*! \brief get the device id of the Atmel Xmega device
 *
 * \retval for  64a3 : 0x00 0x42 0x96 0x1e
 * \retval for 182a3 : 0x00 0x42 0x97 0x1e
 * \retval for 192a3 : 0x00 0x44 0x97 0x1e
 * \retval for 256a3 : 0x00 0x42 0x98 0x1e
 */
uint32_t getXmegaDeviceId();

/*! \brief get the unique serial of the Atmel Xmega device
 *
 * The serial number consist of the production LOT number, wafer number, and wafer coordinates for the device
 */
uint32_t getXmegaSerial();

/*! \brief get the board location through gpio connector setting
 *
 * \retval each board returns a unique value which can be used to select the correct function
 * \retval if 0 then invalid
 */
uint8_t getApplicationId();

uint32_t getCpuClkHz();

bool sendByte( uint8_t value );
bool receiveByteAvailable();
uint8_t receiveBufferSpace();
uint8_t receiveByte();


uint16_t getClockTicks();

void setLedGreen(bool value);
bool getLedGreen();

void setLedYellow(bool value);
bool getLedYellow();

void setPwm( int16_t value );
uint16_t getPwm();
void setEnableGate( bool value );

#endif /* INCLUDED_XMEGA_H */
