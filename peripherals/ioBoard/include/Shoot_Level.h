/*
 *	Module		: Shoot_Level.h
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

#ifndef SHOOT_LEVEL_H_
#define SHOOT_LEVEL_H_

void Shoot_Level_initialize();
void Shoot_Level_home();
void Shoot_Level_set_height(uint8_t height);
void Shoot_Level_set_speed(unsigned char shootSpeed);

#endif /* SHOOT_LEVEL_H_ */
