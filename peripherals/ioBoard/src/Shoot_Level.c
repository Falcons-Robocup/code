/*
 *	Module		: Shoot_Level.c
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls the steppenmotor driver for the Shoot Level.  
 *				  Using an PWM signal for a specific time.  
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


// Include files
#include "config.h"

#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Shoot_Level.h"

// Static variables
volatile static uint8_t bPosition_Old;
volatile static bool fPWM_Busy_Flag;
volatile static int iSteps_puls ;
volatile static bool fhome ;
volatile static bool fhome_toggle;

static unsigned short timerPulsesPerCycle = 150;

#define STEP_MULTIPLYER 4

#define bit_is_set(val, bit_no) (((~val) >> (bit_no)) & 1)
#define home_sensor (bit_is_set(PORTA.IN,5))

static void shoot_level_set(uint8_t bPosition);

// step to home ( up ) puls - puls IO board homesensor
//

/*
 *	Function	: Shoot_Level_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initalisation of the peripherals of the 
 *				  PWM (TCE1) and timer for the length of the PWM signal (TCF0).
 *  
 *	History		:
 *
 */
void Shoot_Level_initialize()
{
	// Set Shoot Level PWM and direction pins to output and low.
	SHOOT_LEVEL_PORT.OUTCLR = SHOOT_LEVEL_PWM_PIN | SHOOT_LEVEL_DIRECTION_PIN;
	SHOOT_LEVEL_PORT.DIRSET = SHOOT_LEVEL_PWM_PIN | SHOOT_LEVEL_DIRECTION_PIN;

	TCF0.CTRLB = 0;					// operate in normal mode
	TCF0.CNT = 0 ;					// Forced counter to count up from 0
	TCF0.INTFLAGS = TC1_OVFIF_bm;	// Clear Interrupts
	TCF0.INTCTRLA= 0x03;			// enable interrupts high level
	fPWM_Busy_Flag = false;

	Shoot_Level_home();
}

	
void Shoot_Level_home()
{
	// shoot level goes max 0xc5 pulsen up or down. Homesensor will stop puls generator when detect earlier.
	fhome = false; // not calibrated yet

	// if magnet is in home postion area then lower the level so it will go outside the home area , then
	// go up and when it comes in the home area the offset will be used to set it in the home position.
	// Go down is around 50 % of height.

	if (home_sensor)
	{
		// go down till sensor is false then go up ...
		fhome_toggle = true;   // set toggle , so level have to go up after its set down  halfway.
		bPosition_Old= 0x00;  // (max up position)
		shoot_level_set(0xA0);  // go to down to half way.
	}
	else
	{
		// go up and then when sensor detect home add offset.
		fhome_toggle = false;
		bPosition_Old= 0xFF;  // new - old value so negative value will go up to home sensor
		shoot_level_set(0x00); // go up.
	}
}

/*
 *	Function	: Interrupt Routine for Overflow of timer used for the shoot puls length.
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: Overflow interrupt vector. 
 *	Return		: none
 *  Description	: The overflow interrupt will be called if the timer overflows. Then the timer is 
 *				  stopped and the interrupt is disabled. The trigger pin is cleared. At the end
 *				  the flag of shoot busy will be cleared, avoiding a trigger will the shoot puls is busy.
 *				  
 *	History		:
 *
 */
ISR(TCF0_OVF_vect)
{	
	// Toggle PWM pin
	SHOOT_LEVEL_PORT.OUTTGL = SHOOT_LEVEL_PWM_PIN;

	// if iSteps = 0 then disable pulse generator so 
	// stepper is not moving anymore.
	if (iSteps_puls == 0)
	{
		
		
		SHOOT_LEVEL_PORT.OUTCLR = SHOOT_LEVEL_PWM_PIN; // reset pin
		TCF0.CTRLA =  0x00 ;   // disable interrupt.
		TCF0.CNT = 0 ;		   // reset timer.
		
		fPWM_Busy_Flag = false ; // clear pwm_busy flag. 
		
		
		// if home is not calibrated yet , set the set flag home_cal true and reset the old position to zero. 
		if ((fhome==false)&&(fhome_toggle==false) )
		{
			
			bPosition_Old= 0xFF;
			//fhome=true;  // homed is done.
			shoot_level_set(0x00); // go up. more till sensor detect..
		}
		
		if ((fhome==false )&&(fhome_toggle==true))
		{
			
			SHOOT_LEVEL_PORT.OUTCLR = SHOOT_LEVEL_DIRECTION_PIN;
			fhome_toggle = false;
			bPosition_Old= 0xFF;  // new - old value so negative value will go up to home sensor
			shoot_level_set(0x00); // go up.
		
		}
	}

	// only when not calibrated  ( fhome= false)
	if((fhome==false)&&(fhome_toggle==false))
	{
		// working go up + offset.
		if (home_sensor)  // detects sensor when go up..
		{
			// if home_sensor == true, stop puls pwm after offset in same direction. keeps on going in same direction

			SHOOT_LEVEL_PORT.OUTCLR = SHOOT_LEVEL_DIRECTION_PIN;  // go up  override ( negative value i stepps to make sure that its go up :)
			iSteps_puls = 4;//HOME_OFFSET ;  // offset extra pulsen go up
			fhome=true;
		} 
	}

	iSteps_puls --;
} 
	
	
/*
 *	Function	: Shoot_Level_Set
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: uint8_t bPosition, This is the value of 0-255 for the level height indication  
 *	Return		: none
 *  Description	: The bPosition (0-255) is the height for the level height. The difference of the new and old value will 
 *				  be the number of steps. This byte will be multiplied 
 *				  by a scale factor constant to scale it from a byte to maximum 3565 pulses. The direction output pin is set 
 *				  high if difference is positive or low when it is negative difference
 *				  The PWM and timer peripheral are started. The PWM will stop when the timer (TCF0) overflows. 
 *				  
 *	History		:
 *
 */	
void shoot_level_set(uint8_t bPosition)
{
	// Only set Level when PWM is not busy yet
	
	if (!fPWM_Busy_Flag)						
	{
		int iSteps = bPosition - bPosition_Old ;		// Calc signed difference
		bPosition_Old= bPosition;					// Save old position
		
		if (iSteps > 0)								// positive difference
		{
			SHOOT_LEVEL_PORT.OUTSET = SHOOT_LEVEL_DIRECTION_PIN;
		}
		else										// negative difference
		{
			SHOOT_LEVEL_PORT.OUTCLR = SHOOT_LEVEL_DIRECTION_PIN;
			iSteps = abs(iSteps) ;					// |iTemp|
		}

		if (iSteps > 0)								// Only start timers when iSteps > 0
		{
			
			TCF0.PER = timerPulsesPerCycle;		// puls lenght for 1 puls 50%    per 1 = 64us        per = x ms / 0.064
			TCF0.CTRLA= 0x04;

			iSteps_puls = (iSteps <<2) * STEP_MULTIPLYER ;  // 2 times _|-					// enable timer   0x07 / pre 1024

			SHOOT_LEVEL_PORT.OUTCLR = SHOOT_LEVEL_PWM_PIN;
			fPWM_Busy_Flag = true ;
		}
	}
}

void Shoot_Level_set_height(uint8_t height)
{
	if (height == 0x00)
	{
		if (!home_sensor)
		{
			Shoot_Level_home();
		}
	}
	else
	{
		shoot_level_set(height);
	}
}

void Shoot_Level_set_speed(unsigned char shootSpeed)
{
	timerPulsesPerCycle = shootSpeed;
}
