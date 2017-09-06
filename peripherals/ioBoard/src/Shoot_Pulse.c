/*
 * Shoot_Pulse.c
 *
 *  Created on: Jul 11, 2017
 *      Author: Edwin Schreuder
 */

#include "config.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "Charge_Pulse.h"

// Static variables
static bool shoot_busy = false;

//! Initializes the Shoot Pulse module.
//! \details	Initialization of the trigger i/o pin and the timer TCE0 as normal
//!				timer with interrupt.
void Shoot_Pulse_initialize()
{
	// Configure trigger pin as output.
	SHOOT_TRIGGER_PORT.OUTCLR = SHOOT_TRIGGER_PIN;
	SHOOT_TRIGGER_PORT.DIRSET = SHOOT_TRIGGER_PIN;

	TCE0.CTRLB = 0;					// operate in normal mode
	TCE0.CNT = 0;					// Forced counter to count up from 0
	TCE0.INTFLAGS = TC1_OVFIF_bm;	// Clear Interrupts
	TCE0.INTCTRLA= 0x03;			// enable interrupts high level

	shoot_busy = false;				// Clear Flag
}

//! Trigger the shoot pulse.
//! \details	When the trigger is set with a specific
//!				time the pin goes high for this period. When the Timer TEC0 overflows the interrupt
//!				will stop the timer and reset the pin. There is only one shot at the time.
void Shoot_Pulse_trigger(uint8_t time)
{
	// Only trigger one shoot pulse at the time
	if (!shoot_busy)
	{
		shoot_busy = true;
		// set top value, prescaler is 1024 and CLK is 16MHz. 
		// 0- 255 -> 0 - 32.5 ms. The time of 255 counts is 16.25 ms, so a 
		// multiply by 2 * time is needed to get  it to 32.5ms 
		TCE0.PER = time << 1 ;	

		// Set the shoot trigger pin high and start the timer (prescaler = 1024)
		SHOOT_TRIGGER_PORT.OUTSET = SHOOT_TRIGGER_PIN;
		TCE0.CTRLA =  0x07;
	}
}

//! Interrupt service routine for the timer used to determine the shoot pulse length.
//! \details	The overflow interrupt will be called if the timer overflows.
//!				At that point, the timer is stopped and the interrupt is disabled. The trigger pin is cleared.
//!				At the end the flag of shoot busy will be cleared, avoiding a trigger will the shoot pulse is busy.
ISR(TCE0_OVF_vect)
{
	// Reset the pin.
	SHOOT_TRIGGER_PORT.OUTCLR = SHOOT_TRIGGER_PIN;

	// Disable the interrupt.
	TCE0.CTRLA = 0x00;

	// Reset the timer.
	TCE0.CNT = 0;

	// Reset the flag to enable shoot again.
	shoot_busy = false;

	// Reinitialize the charge pulse so we recover the shooter immediately after shooting.
	Charge_Pulse_initialize();
}
