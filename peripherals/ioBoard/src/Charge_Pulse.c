/*
 * Charge_Pulse.c
 *
 * Created: 3/29/2016 8:51:18 PM
 *  Author: Peter Bradley + Tim Kouters
 */ 

#include "config.h"

#include <stdbool.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "Charge_Pulse.h"

static uint8_t NR_CHARGE_PULSE_HIGH = 10;
static uint16_t pulseCounter = 0;

void Charge_Pulse_initialize()
{
	// Configure trigger pin as output.
	SHOOT_CHARGE_PORT.DIRSET = SHOOT_CHARGE_PIN;

	// Stop the timer.
	TCE1.CTRLA =  0x00;

	// Reset the pulseCounter to zero such that a pulse is generated immediately
	// when the timer elapses.
	pulseCounter = 0;

	// Initialize the timer and interrupts.
	TCE1.CTRLB = 0;					// Operate in normal mode
	TCE1.INTFLAGS = TC1_OVFIF_bm;	// Clear Interrupts
	TCE1.INTCTRLA = 0x03;			// Enable interrupts high level

	// Set top value, prescaler is 1024 and CLK is 16MHz.
	// So 16000000 / 1024 = 15625 period ticks per second
	// Setting the period to 15625 will give an ISR per second
	TCE1.PER = 15625;
	TCE1.CNT = TCE1.PER - 1;		// Force counter to elapse immediately
	TCE1.CTRLA =  0x07;				// Start enable timer, prescaler = 1024
}

ISR(TCE1_OVF_vect)
{
	if (pulseCounter == 0)
	{
		SHOOT_CHARGE_PORT.OUTCLR = SHOOT_CHARGE_PIN; // reset pin
		pulseCounter = NR_CHARGE_PULSE_HIGH;
	}
	else
	{
		SHOOT_CHARGE_PORT.OUTSET = SHOOT_CHARGE_PIN;
	}

	// Decrement the pulseCounter
	pulseCounter--;
}
