
/*
 *	Module		: ADC.c
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls the ADC for reading the analog input
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

// needed for adc calibration
#include <stddef.h>
#include <avr/pgmspace.h>

static uint8_t read_calibration_byte(uint8_t);

/* 
 *	Function	: ADC_Init
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Initialisation of the ADC peripheral to measured a analog signal. ref voltage is the internal 1V.
 *  
 *	History		:
 *
 */
void ADC_initialize()
{
	ADCB.CALL = read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL0));
	ADCB.CALH = read_calibration_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCBCAL1));

	PORTB.DIR = 0x00 ; // set pinb 0-7 as input
	ADCB.CTRLB= ADC_RESOLUTION_12BIT_gc;

	ADCB.CTRLA|= 0x01;  // enable ADC port B
	//ADCB.CTRLB= 0x08;	// unsigned , free running, 12 bit right adjusted
	ADCB.CTRLB= ADC_RESOLUTION_12BIT_gc;

	ADCB.REFCTRL = ADC_REFSEL_AREFB_gc ;   // use external 2.5 v op portb ref 
	//ADCB.REFCTRL = ADC_REFSEL_AREFB_gc;

	//  ADCB.EVCTRL = 0xC0 ;  // sweep all channels no event
	//can be zero ?
	ADCB.PRESCALER= ADC_PRESCALER_DIV8_gc; // prescaler = 8

	ADCB.INTFLAGS=0x0F;  // clear all adc channels interrrupts
	
	//channels (all channels sweep in running mode )
	ADCB.CH0.CTRL=  ADC_CH_INPUTMODE_SINGLEENDED_gc;  // single ended positive input, no start ..
	ADCB.CH0.MUXCTRL= ADC_CH_MUXPOS_PIN1_gc; //select Pb1 as ch0 input.

	ADCB.CH1.CTRL=  ADC_CH_INPUTMODE_SINGLEENDED_gc;  // single ended positive input, no start ..
	ADCB.CH1.MUXCTRL= ADC_CH_MUXPOS_PIN2_gc; //select Pb2 as ch1 input.

	ADCB.CH2.CTRL=  ADC_CH_INPUTMODE_SINGLEENDED_gc;  // single ended positive input, no start ..
	ADCB.CH2.MUXCTRL= ADC_CH_MUXPOS_PIN3_gc; //select Pb3 as ch2 input.

	ADCB.CH3.CTRL=  ADC_CH_INPUTMODE_SINGLEENDED_gc;  // single ended positive input, no start ..
	ADCB.CH3.MUXCTRL= ADC_CH_MUXPOS_PIN4_gc; //select Pb4 as ch3 input.

	_delay_ms(1);  // delay ADC start up
}
/*
 *	Function	: ADCB_GET
 *  Author		: Dirk-Jan Vethaak
 *  Parameters	: none
 *	Return		: none
 *  Description	: Start ADC conversion. 
 *				  Data is read in Main  by abBuffer_tx[3] = ADCB.CH0RESH;
 *										   abBuffer_tx[4] = ADCB.CH0RESL;	
 *  
 *	History		:
 *
 */
void ADC_GET()
{	
		ADCB.CH0.CTRL |=ADC_CH_START_bm;  // single ended positive input+start read from adc
		while(!ADCB.CH0.INTFLAGS); // wait till conversion is complete
		//ADCB.CH0.INTFLAGS=0x01  ;// clear bit
		
		
		//  ADCB.CH1.CTRL |=ADC_CH_START_bm;  // single ended positive input+start read from adc
		// while(!ADCB.CH1.INTFLAGS); // wait till conversion is complete
		// ADCB.CH1.INTFLAGS=0x01  ;// clear bit
		
		ADCB.CH1.CTRL |=ADC_CH_START_bm;  // single ended positive input+start read from adc
		while(!ADCB.CH1.INTFLAGS);
		
		ADCB.CH2.CTRL |=ADC_CH_START_bm;  // single ended positive input+start read from adc
		while(!ADCB.CH2.INTFLAGS);
		
		ADCB.CH3.CTRL |=ADC_CH_START_bm;  // single ended positive input+start read from adc
		while(!ADCB.CH3.INTFLAGS);
}

uint8_t read_calibration_byte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}

