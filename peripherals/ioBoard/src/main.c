/*
 * main.c
 *
 * Description: Heavily cleaned up version of IO board firmware.
 *  Created on: Jun 8, 2017
 *      Author: Edwin Schreuder
 */

#include "config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "Charge_Pulse.h"
#include "Digital_Input.h"
#include "Package.h"
#include "Shoot_Level.h"
#include "Shoot_Pulse.h"
#include "UART.h"

// Function Prototype 
static void setup();
static void setup_clock();
static void setup_watchdog_timer();
static void loop();

static void command_get_version(uint8_t payload[]);
static void command_set_shoot(uint8_t payload[]);
static void command_set_home(uint8_t payload[]);
static void command_set_lever_speed(uint8_t payload[]);
static void command_set_height(uint8_t payload[]);
static void command_get_status(uint8_t payload[]);
static void command_set_bootloader(uint8_t payload[]);

//! Main entry point of the application.
//! \details	The main function will first call all init functions. Then it will stay n the main loop.
//!				when it received a packet then the main function will perform the action for that command. After
//!				that it will send  a transmit packet to the host.
int main(void)
{	
	setup();

	loop();
}

//! Performs the initial setup.
//! \details	All the initialization function calls and global interrupt enable are
//!				put together in this function.
void setup()
{
	// Initialize the clock.
	setup_clock();

	// Delay some time to allow hardware to start up and settle.
	_delay_ms(500);

	// Initialize the main UART.
	UART_initialize();

	// Initialize the digital inputs.
	Input_initialize();

	// Initialize the shoot pulse.
	Shoot_Pulse_initialize();

	// Initialize the charge pulse.
	Charge_Pulse_initialize();

	// Initialize the shoot level.
	Shoot_Level_initialize();

#if WATCHDOG_TIMER_ENABLED
	setup_watchdog_timer();
#endif	

	PMIC.CTRL = 0x07;	// interrupt levels high
	sei();				// general interrupts enabled

}

//! Initialize the clock with the correct settings.
//! \\details	The clock will be configured to a external crystal of 16 MHz.
void setup_clock()
{
	OSC.XOSCCTRL = 0xCB;	// select extern crystal
	OSC.CTRL |= OSC_XOSCEN_bm;

	while (!(OSC.STATUS & OSC_XOSCRDY_bm)); //wait until stable

	CCP = 0xD8;		// allow to write protected register
	CLK.CTRL = 0x03;  // select extern crystal

	CCP = 0xD8;		// allow to write protected register
	CLK.PSCTRL = 0;
}

void setup_watchdog_timer()
{
	wdt_enable(WATCHDOG_TIMER_PERIOD);
}

void loop()
{
	Package package;

	while (true)  //super loop
	{
		if (Package_receive(&package))
		{
			// Select action from bCommand
			switch (package.command)
			{
			case CMD_SET_SHOOT:
				command_set_shoot(package.payload);
				break;
			case CMD_SET_HOME:
				command_set_home(package.payload);
				break;
			case CMD_SET_LEVER_SPEED:
				command_set_lever_speed(package.payload);
				break;
			case CMD_SET_HEIGHT:
				command_set_height(package.payload);
				break;
			case CMD_GET_STATUS:
				command_get_status(package.payload);
				break;
			case CMD_GET_VERSION:
				command_get_version(package.payload);
				break;
			case CMD_SET_BOOTLOADER:
				command_set_bootloader(package.payload);
				break;
			default:
				package.command = CMD_ERROR;
				break;
			}

			Package_send(&package);
		}

#if WATCHDOG_TIMER_ENABLED
		wdt_reset();
#endif
	}
}

void command_get_version(uint8_t payload[])
{
	memcpy(payload, VERSION, PACKAGE_PAYLOAD_LENGTH);
}

void command_set_shoot(uint8_t payload[])
{
	// Trigger a shoot.
	Shoot_Pulse_trigger(payload[0]);
}

void command_set_home(uint8_t payload[])
{
//	Shoot_Level_home();
}

void command_get_status(uint8_t payload[])
{
	payload[0] = Input_isInPlay();
	payload[1] = Input_isSoftwareOn();
}

void command_set_lever_speed(uint8_t payload[])
{
	Shoot_Level_set_speed(payload[0]);
}

void command_set_height(uint8_t payload[])
{
	// Set shoot level
	Shoot_Level_set_height(payload[0]);
}

//! Starts the bootloader.
//! \details	Issue a software reset; the bootloader will detect the software was reset and go into bootloader mode.
//! \author	Edwin Schreuder
static void command_set_bootloader(uint8_t payload[])
{
	Package package;
	package.command = CMD_SET_BOOTLOADER;
	Package_send(&package);

	// Unlock the reset register such that a software rest can be issued.
	CCP = CCP_IOREG_gc;

	// Issue a software reset, this will automatically start the bootloader.
	RST.CTRL = RST_SWRST_bm;
}
