// Copyright 2015 Jan Feitsma & Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include "spiMaster.h"
#include "sysclk.h"
#include "xmega.h"

static uint16_t error = DRV8301_ERROR_INIT_NOT_PERFORMED;
static drv8301Type drv8301Val;

// prototypes
void getDrv8301All( );
void checkDrv8301Registers( );
void setDrv8301ControlRegisters( );

// Setup the drv8301 motor driver through the spi interface.
// After power up the drv8301 is configured for 6 PWM inputs while the board has a 6 PWM input.
// So without this configuration the motor will not get any power.
void initSpiMaster() {
	// clear the uninitialized flag
	error = 0; // clear the initialization flag

	// based on the following xmega spi example
	// https://eewiki.net/display/microcontroller/SPI+Example+for+Xmega

	// same bank as spi, configured in spiMaster.c #define USARTRX IOPORT_CREATE_PIN(PORTC, 2)
	// same bank as spi, configured in spiMaster.c #define USARTTX IOPORT_CREATE_PIN(PORTC, 3)

	// configure spi port interface
	//   7   6    5    4      3         2    1   0
	//  sck miso mosi  cs  usarttx  usartrx  nc  nc
	PORTC.DIR = 0xB8; // mosi, sck, ss and usarttx as outputs; miso and usartrx as input

	sysclk_enable_module(SYSCLK_PORT_C, PR_SPI_bm); // this has to do with using the 32MHz clock as spi clock source

	// from drv8301 datasheet :
	// SLCK idles low, data is setup on resing edge SCLK, data is sampled on falling edge SCLK = spi mode 1
	// SCLK clock maximal 10MHz (100ns)
	// SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE0_bm; // double speed, msb first, enable, master mode, spi mode 1, no prescaler
	// no SPI_PRESCALER0_bm period is 122ns ~ 8MHz (when system clock is set to 32MHz)
	// from measurement SPI_PRESCALER0_bm is divide by 4 about 492 ns ~ 2MHz
	// from measurement SPI_PRESCALER1_bm is divide by 16 about 1968 ns ~ 500kHz
	// for double speed use SPI_CLK2X_bm
	SPIC.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE0_bm | SPI_PRESCALER0_bm; // msb first, enable, master mode, spi mode 1, prescaler to get 2MHz

	// set the values we need to be able to use the drv8301 motor driver
	// TODO: checkout if the values are correct for the wheel motors, especially the OC_ADJ_SET
	drv8301Val.setControl1 = 0;
	drv8301Val.setControl1 |=  0<<0; // GATE_CURRENT : Gate drive peak current 1.7 A
	drv8301Val.setControl1 |=  1<<2; // GATE_RESET : Reset gate driver latched faults (reverts to 0)
	drv8301Val.setControl1 |=  1<<3; // PWM_MODE : 3 PWM inputs
	drv8301Val.setControl1 |=  0<<4; // OCP_MODE : Current limit
	drv8301Val.setControl1 |= 10<<6; // OC_ADJ_SET : Over current Adjustment = 197mV
	drv8301Val.setControl2 = 0;
	drv8301Val.setControl2 |= 0<<0; // OCTW_MODE : Report both over temperature (OT) and over current (OC) at nOCTW pin
	drv8301Val.setControl2 |= 0<<2; // GAIN : Gain of shunt amplifier: 10 V/V
	drv8301Val.setControl2 |= 0<<4; // DC_CAL_CH1 : Shunt amplifier 1 connects to load through input pins
	drv8301Val.setControl2 |= 0<<5; // DC_CAL_CH2 : Shunt amplifier 2 connects to load through input pins
	drv8301Val.setControl2 |= 0<<6; // OC_TOFF : Cycle by cycle = the MOSFET on which over current has been detected on will shut off until the next PWM cycle

	// write the control data to the drv8301 motor controller
	setDrv8301ControlRegisters( );
}

// run on a regular basis
void taskSpiMaster() {
	// read all 4 registers from the drv8301
	getDrv8301All( );
	// check the values of the 4 registers
	checkDrv8301Registers( ); // the check functions set one or more error flags in case of an error
}

// return value is response on previous read or write from drv8301
inline uint16_t spi8301Write(uint8_t addr, uint16_t data) {
	addr = addr & 0x1f; // address is 5 bits
	data = data & 0x07ff; // data is 11 bits

	PORTC.OUTCLR = 0x10; // assert SS pin (active low)

	SPIC.DATA = 0<<7 | addr<<3 | data>>8; // send first byte with: write bit, address and 3 bits msb data
	while(!(SPIC.STATUS & 0x80)); // wait for transmit complete
	uint16_t response = SPIC.DATA<<8; // 8 bits msb data response from the previous read or write

	SPIC.DATA = data & 0xff; // send second byte with 8 bits lsb data
	while(!(SPIC.STATUS & 0x80)); // wait for transmit complete
	response |= SPIC.DATA; // 8 bits lsb data response from the previous read or write

	PORTC.OUTSET = 0x10; // de-assert SS pin

	return response;
}

// this sends the read command to the drv8301, the data will be returned in the next read or write
// return value is response on previous read or write from drv8301
inline uint16_t spi8301ReadCmd(uint8_t addr) {
	addr = addr & 0x1f; // address is 5 bits

	PORTC.OUTCLR = 0x10; // assert SS pin (active low)

	SPIC.DATA = 1<<7 | addr<<3; // send first byte with: write bit and address
	while(!(SPIC.STATUS & 0x80)); // wait for transmit complete
	uint16_t response = SPIC.DATA<<8; // 8 bits msb data response from the previous read or write

	SPIC.DATA = 0x00; // send second byte
	while(!(SPIC.STATUS & 0x80)); // wait for transmit complete
	response |= SPIC.DATA; // 8 bits lsb data response from the previous read or write

	PORTC.OUTSET = 0x10; // de-assert SS pin

	return response;
}

// read all 4 registers in a row to so only 5 cycles are need instead of 8
void getDrv8301All( ) {
	spi8301ReadCmd(0); // ask drv8301 for status register 1, response is if of previous command, so not relevant
	drv8301Val.status1 = spi8301ReadCmd(1); // ask drv8301 for status register 2, response is if of previous command, which is status register 1
	drv8301Val.status2 = spi8301ReadCmd(2); // ask drv8301 for control register 1, response is if of previous command, which is status register 2
	drv8301Val.getControl1 = spi8301ReadCmd(3); // ask drv8301 for control register 2, response is if of previous command, which is control register 1
	drv8301Val.getControl2 = spi8301Write(0, 0); // perform dummy write (write to read only address), response is if of previous command, which is control register 2
}

// check the value of drv8301 status registers and control registers
void checkDrv8301Registers( ) {
	// check the value of status register 1 (status register for device faults)
	if( ( (drv8301Val.status1>>0) & 1 ) == 1 ) { error |= DRV8301_ERROR_FETLC_OC; }
	if( ( (drv8301Val.status1>>1) & 1 ) == 1 ) { error |= DRV8301_ERROR_FETHC_OC; }
	if( ( (drv8301Val.status1>>2) & 1 ) == 1 ) { error |= DRV8301_ERROR_FETLB_OC; }
	if( ( (drv8301Val.status1>>3) & 1 ) == 1 ) { error |= DRV8301_ERROR_FETHB_OC; }
	if( ( (drv8301Val.status1>>4) & 1 ) == 1 ) { error |= DRV8301_ERROR_FETLA_OC; }
	if( ( (drv8301Val.status1>>5) & 1 ) == 1 ) { error |= DRV8301_ERROR_FETHA_OC; }
	if( ( (drv8301Val.status1>>6) & 1 ) == 1 ) { error |= DRV8301_ERROR_OTW; }
	if( ( (drv8301Val.status1>>7) & 1 ) == 1 ) { error |= DRV8301_ERROR_OTSD; }
	if( ( (drv8301Val.status1>>8) & 1 ) == 1 ) { error |= DRV8301_ERROR_PVDD_UV; }
	if( ( (drv8301Val.status1>>9) & 1 ) == 1 ) { error |= DRV8301_ERROR_GVDD_UV; }
	if( ( (drv8301Val.status1>>10) & 1 ) == 1 ) { error |= DRV8301_ERROR_FAULT; }

	// check the value of status register 2 (status register for device faults and ID)
	if( ( (drv8301Val.status2>>0) & 0xf ) != 1 ) { error |= DRV8301_ERROR_DEVICE_ID; } // the device ID of drv8301 is always 1
	if( ( (drv8301Val.status2>>7) & 1 ) == 1 ) { error |= DRV8301_ERROR_GVDD_OV; }

	// check the value of control register 1 (gate driver control)
	uint16_t expected = (2<<11) | ( drv8301Val.setControl1 & 0x07fb ); // the b in 0x07fb because that value get's reset to zero after write
	if( drv8301Val.getControl1 != expected ) { error |= DRV8301_ERROR_CONTROL_REGISTER1; }

	// check the value of control register 2 (current shunt amplifiers and misc control)
	expected = (3<<11) | drv8301Val.setControl2;
	if( drv8301Val.getControl2 != expected ) { error |= DRV8301_ERROR_CONTROL_REGISTER2; }
}

// write value to drv8301 control registers
void setDrv8301ControlRegisters( ) {
	// gate driver control
	spi8301Write(2, drv8301Val.setControl1);
	// current shunt amplifiers and misc control
	spi8301Write(3, drv8301Val.setControl2);
}

// get the static local variables (which are updated on a regular basis) representing the status of the drv8301
drv8301Type getDrv8301( ) {
	// reading the values from the drv8301 takes a while, but it is undesired to create additional delay on the
	// communication link, therefore use the cached values, which are updated on a regular base by the taskSpiMaster
	return drv8301Val;
}

// update the control registers of the drv8301
void setDrv8301( uint16_t control1, uint16_t control2 ) {
	drv8301Val.setControl1 = control1;
	drv8301Val.setControl2 = control2;
	setDrv8301ControlRegisters( );
	// the taskSpiMaster will verify if the values are written correctly
}

// return the error vector of the motor driver
uint16_t getSpiMasterError() {
	return error;
}

// clear one or more bits in the motor driver error vector
void clearSpiMasterError(uint16_t value) {
	error &= ~value; // clear only the bits set in value
}

