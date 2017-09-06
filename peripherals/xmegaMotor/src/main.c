// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include "communication.h"
#include "encoder.h"
#include "global.h"
#include "pid.h"
#include "pwm.h"
#include "safety.h"
#include "scheduler.h"
#include "spiMaster.h"
#include "xmega.h"
#include "genericFilter2nd.h"

pidStateT anglePID; // only used in ball hander mode
pidStateT primaryPID; // used for wheel motor and ball handler tacho
genericFilter2ndValues angleLowPass;
genericFilter2ndValues primaryLowPass;

int main(int argc, char** argv)
{
	// initialization
	initXmega(); // this one first because it sets up the io and calls sysclk_init
	initCommunication();
	initEncoder();
	initPid(&anglePID);
	initPid(&primaryPID);
	initGenericFilter2nd(&angleLowPass);
	initGenericFilter2nd(&primaryLowPass);
	initPwm();
	initSpiMaster();
	initScheduler();
	initSafety();

	while( true ) {
		// use all spare cpu time to process the received data from the far end
		// and to send back the last status of the board (for localization and diagnostics).
		taskCommunication();
	}
	return 0;
}

#ifdef NONO
#include <avr/io.h>

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

int main(void)
{
    uint16_t response;
    uint16_t failure = 0;

    // checkout /opt/avr_v3.4.5/avr/include/avr/iox64a3.h for bit definitions

    // both following do not make a difference
    // OSC.CTRL = OSC_RC32MEN_bm;
    // CLK.CTRL = CLK_SCLKSEL2_bm | CLK_SCLKSEL1_bm;


    // configure drv8301 gate enable and led pins
    //  7    6     5        4       3    2     1   0
    //  nc  nc ledGreen ledYellow 422tx 422rx en_g nc
    PORTF.DIR = 0x32; // ledGreen, ledYellow and en_g output

    // configure spi port interface
    //   7   6    5    4   3   2   1   0
    //  sck miso mosi  cs  nc  nc  nc  nc
    PORTC.DIR = 0xB0; // mosi, sck, ss outputs; miso input

    // from drv8301 datasheet :
    // SLCK idles low, data is setup on resing edge SCLK, data is sampled on falling edge SCLK = spi mode 1
    // SCLK clock maximal 10MHz (100ns)
    SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE0_bm; // double speed, msb first, enable, master mode, spi mode 1, no prescaler
    // oscilloscope measurement shows 1000ns (1MHz, probably system clock clock 2MHz)

    // flush the recieve buffer of the master
    while(SPIC.STATUS & 0x80) {	response = SPIC.DATA; }

    PORTF.OUTSET = 0x02; // enable drv8301 by asserting enable gate pin

	uint8_t addr = 0;
	uint16_t data = 0;
    while(1) {
    	// write to dr8301
    	response = spi8301Write(addr, data);
    	if( response != 0 ) { failure = 5000; } // response is status register last dummy read (end of this loop)

    	// send read command to drv8301
    	response = spi8301ReadCmd(addr); // response is status register from previous write (above)
    	if( response != 0 ) { failure = 5000; }

    	// collect read response from drv8301
    	response = spi8301Write(0, 0); // collect the read data from the previous read by doing a dummy write (write to read only address)

    	// check the response from drv8301
    	bool good = 0;
		addr = addr & 0x1f; // address is 5 bits
    	data = data & 0x07ff; // data is 11 bits
		uint16_t expected = addr<<11; // response always includes address (and failure bit on msb bit)
		if( addr == 0 ) {
			expected |= 0x0000;
			if( response == expected ) { good = 1; } else { failure = 5000; }
    	} else if( addr == 1 ) {
    		expected |= 0x0801;
    		if( response == expected ) { good = 1; } else { failure = 5000; }
		} else if( addr == 2 ) {
    		expected |= data & 0x07fb; // the b in 0x07fb because that value get's reset to zero after write
    		if( response == expected ) { good = 1; } else { failure = 5000; }
    	} else if( addr == 3 ) {
    		expected |= data;
    		if( response == expected ) { good = 1; } else { failure = 5000; }
    	} else if( addr == 4 ) {
    		expected |= data & 0x000f;
    		if( response == expected ) { good = 1; } else { failure = 5000; }
    	} else {
    		expected |= 0x0000;
    		if( response == expected ) { good = 1; } else { failure = 5000; }
    	}
        if( good ) { PORTF.OUTSET = 0x20; } else { PORTF.OUTCLR = 0x20; } // led1 (green) on when received data unequal zero

    	// if( ii == 5000 ) { PORTF.OUTSET = 0x10; } else if ( ii == 10000 ) { PORTF.OUTCLR = 0x10; ii = 0; } // toggle led2 (yellow)
    	if( failure == 0 ) {
    		PORTF.OUTCLR = 0x10;
    	}  else {
    		PORTF.OUTSET = 0x10; // show failure on led2
    		failure--;
    	}
    	data += 321;
    	if( addr > 4 ) { addr = 0; } else { addr++; }
    }
}

// reading from (the upper 5 bits are returned from read command: read and address)
// address 0 0x0000 : status register 1 (all defaults are zero) : read, address 0
// address 1 0x0801 : status register 2 (all defaults are zero) : read, address 1 (the 8), device id = 1
// address 2 0x1400 : control register 1 for gate driver : read, address 2 (the 1), 1.7A, gate reset normal mode, 6 pwm inputs, current limmit, overcurrent adj 16 = 0.403
// address 3 0x1800 : control register 2 for shunt amplifier : read, address 3 (the 18), OT and OC at nOCTW pin, gain shunt amplifier 10v/v, shunt anmp1 connected to load through inputs, shunt anmp2 connected to load through inputs,

#endif
