/*
 * Keeper_Frame_Pulse.c
 *
 *  Created on: Dec 18, 2018
 *      Author: Sofia Ntella
 */

#include "config.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>


// Static variables
static bool keeper_frame_busy = false;
static int actuator_flag;

//! Initializes the Keeper Frame Pulse module.
//! \details    Initialization of the trigger i/o pin and the timer TCE0 as normal
//!                timer with interrupt.
void Keeper_Frame_Pulse_initialize()
{
    // Configure trigger pins as outputs.
    KEEPER_FRAME_TRIGGER_PORT.OUTSET |= KEEPER_FRAME_TRIGGER_RIGHT_PIN + KEEPER_FRAME_TRIGGER_UP_PIN + KEEPER_FRAME_TRIGGER_LEFT_PIN;
    KEEPER_FRAME_TRIGGER_PORT.DIRSET |= KEEPER_FRAME_TRIGGER_RIGHT_PIN + KEEPER_FRAME_TRIGGER_UP_PIN + KEEPER_FRAME_TRIGGER_LEFT_PIN;

    TCC1.CTRLB = 0;                    // operate in normal mode-----------> inactive bus timeout DISABLED
    TCC1.CNT = 0;                    // Forced counter to count up from 0-----------> initial counter value
    TCC1.INTFLAGS = TC1_OVFIF_bm;                    // Clear Interrupts--------------> interrupt flag register
    TCC1.INTCTRLA = 0x03;                    // enable interrupts high level--------> an interrupt of a higher level (priority) will stop any running lower level interrupt
    TCC1.CTRLA = 0x06;                               // Set prescaler to 256

    keeper_frame_busy = false;            // Clear Flag
}

//! Trigger the keeper frame pulse.
//! \details    When the trigger is set with a specific
//!                time the pin goes high for this period. When the Timer TEC0 overflows the interrupt
//!                will stop the timer and reset the pin. There is only one keeper frame actuator extension at the time.
void Keeper_Frame_Pulse_trigger_right()
{
    // Only trigger one keeper frame pulse at the time
    if (!keeper_frame_busy)
    {
        actuator_flag = 1;
        keeper_frame_busy = true;
        // set top value, prescaler is 256 and CLK is 16MHz. (1/16000000)*256 per count
        // 0- 62500 counts = 1 sec 
        TCC1.PER = 0xF424 ;    //--------->this is the period and is equal to the top value, till which the timer is counting

        // Set the keeper frame trigger right pin high and start the timer (prescaler = 1024)
        KEEPER_FRAME_TRIGGER_PORT.OUTCLR = KEEPER_FRAME_TRIGGER_RIGHT_PIN;
        TCC1.CTRLA =  0x06; // ----------> enable timer (high resolution plus ENABLED, Timer/Counter 1 ENABLED)
    }
}

void Keeper_Frame_Pulse_trigger_up()
{
    // Only trigger one keeper frame pulse at the time
    if (!keeper_frame_busy)
    {
        actuator_flag = 2;
        keeper_frame_busy = true;
        // set top value, prescaler is 256 and CLK is 16MHz. (1/16000000)*256 per count
        // 0- 62500 counts = 1 sec 
        TCC1.PER = 0xF424 ;    //--------->this is the period and is equal to the top value, till which the timer is counting

        // Set the keeper frame trigger up pin high and start the timer (prescaler = 1024)
        KEEPER_FRAME_TRIGGER_PORT.OUTCLR = KEEPER_FRAME_TRIGGER_UP_PIN;
        TCC1.CTRLA =  0x06;
    }
}

void Keeper_Frame_Pulse_trigger_left()
{
    // Only trigger one keeper frame pulse at the time
    if (!keeper_frame_busy)
    {
        actuator_flag = 3;
        keeper_frame_busy = true;
        // set top value, prescaler is 256 and CLK is 16MHz. (1/16000000)*256 per count
        // 0- 62500 counts = 1 sec 
        TCC1.PER = 0xF424 ;    //--------->this is the period and is equal to the top value, till which the timer is counting

        // Set the keeper frame trigger left pin high and start the timer (prescaler = 1024)
        KEEPER_FRAME_TRIGGER_PORT.OUTCLR = KEEPER_FRAME_TRIGGER_LEFT_PIN;
        TCC1.CTRLA =  0x06;
    }
}

//! Interrupt service routine for the timer used to determine the keeper frame pulse length.
//! \details    The overflow interrupt will be called if the timer overflows.
//!                At that point, the timer is stopped and the interrupt is disabled. The trigger pin is cleared.
//!                At the end the flag of keeper frame busy will be cleared, avoiding a trigger will the keeper frame pulse is busy.
ISR(TCC1_OVF_vect)
{
    // Reset the pin.
    if (actuator_flag == 1)
    {
        KEEPER_FRAME_TRIGGER_PORT.OUTSET = KEEPER_FRAME_TRIGGER_RIGHT_PIN;
    }
    if (actuator_flag == 2)
    {
        KEEPER_FRAME_TRIGGER_PORT.OUTSET = KEEPER_FRAME_TRIGGER_UP_PIN;
    }

    if (actuator_flag == 3)
    {
        KEEPER_FRAME_TRIGGER_PORT.OUTSET = KEEPER_FRAME_TRIGGER_LEFT_PIN;
    }

    // Disable the interrupt.
    TCC1.CTRLA = 0x00;

    // Reset the timer.
    TCC1.CNT = 0;

    // Reset the flag to enable keeper frame extend again.
    keeper_frame_busy = false;
}
