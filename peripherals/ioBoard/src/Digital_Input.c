/*
 *	Module		: Digital_Input.c
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls and read the digital Input data pins.   
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include "config.h"

#include <avr/io.h>

/*
 *	Function	: Input_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initalisation of digital input pins and address pin connected to the dipswitches.
 *				  
 *	History		:
 *
 */
void Input_initialize()
{
	PORTA.OUTCLR = 0xFF;	// Clear data pins, no pull up.
	PORTA.DIRCLR = 0xFF;	// Set data pins as input

	PORTB.OUTCLR = 0xFF;	// Clear data pins, no pull up.
	PORTB.DIRCLR = 0xFF;	// Set data pins as input

	PORTD.DIRCLR = 0xF3;	// Set as input used for dipswitches and in 7 and in 8 ~
}


/*
/*
 *	Function	: getInOutPlay
 *  Author		: Tim Kouters
 *  Parameters	: none
 *	Return		: InOut Play  
 *  Description	: Returns the In/Out play pin value on portA. The data is pulled up at the board. So the data will be 
 *				  inverted to get an '1' for active , '0' for not active.
 *				  
 *	History		: 26-05-2015 Initial creation
 *
 */
bool Input_isInPlay()
{
	uint8_t inv_partA = PORTA.IN ^ 0xFF;
	uint8_t inoutplay_value = inv_partA & INOUT_PLAY_pm;
	uint8_t inoutplay_shifted = inoutplay_value >> INOUT_PLAY_shr;
	return inoutplay_shifted;  
}

//! Retrieves the status of the Software On/Off Switch
//! \details	Returns 1 when the Software On/Off Switch is in the 'On' position.
//!		Returns 0 when the Software On/Off Switch is in the 'Off' position.
//! \author	Edwin Schreuder
bool Input_isSoftwareOn()
{
	return ((SOFTWARE_ON_PORT.IN & SOFTWARE_ON_PIN) == 0);
}
