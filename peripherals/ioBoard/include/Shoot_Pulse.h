/*
 *	Module		: Shoot_Pulse.h
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls the trigger puls for the Shoot module in the Turtle 5K for a specific time. 
 *				  0- 30 ms  
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#ifndef INCLUDE_SHOOT_PULSE_H_
#define INCLUDE_SHOOT_PULSE_H_

// Initialize the Shoot Pulse module.
void Shoot_Pulse_initialize();

// Triggers the shoot puls module to shoot
void Shoot_Pulse_trigger(uint8_t time);

#endif /* INCLUDE_SHOOT_PULSE_H_ */
