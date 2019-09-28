/*
 *	Module		: Keeper_Frame_Pulse.h
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 18-12-2018
 *  Description	: Controls the trigger pulse for the Keeper frame actuators in the Turtle 5K for a specific time. 
 *				  0- 30 ms  
 *  Author		: Sofia Ntella
 *
 *	History		:
 *
 */

#ifndef INCLUDE_KEEPER_FRAME_PULSE_H_
#define INCLUDE_KEEPER_FRAME_PULSE_H_

// Initialize the Keeper Frame pulse module
void Keeper_Frame_Pulse_initialize();

// Triggers the Keeper frame actuator to extend right
void Keeper_Frame_Pulse_trigger_right();

// Triggers the Keeper frame actuator to extend up
void Keeper_Frame_Pulse_trigger_up();

// Triggers the Keeper frame actuator to extend left
void Keeper_Frame_Pulse_trigger_left();

#endif /* INCLUDE_KEEPER_FRAME_PULSE_H_ */
