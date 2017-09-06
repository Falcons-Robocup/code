/*
 *	Module		: Digital Input.h
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 02-07-2013
 *  Description	: Controls and read the digital Input data pins.   
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */


#ifndef DIGITAL_INPUT_H_
#define DIGITAL_INPUT_H_

#include <stdbool.h>

void Input_initialize();

bool Input_isInPlay();
bool Input_isSoftwareOn();


#endif /* DIGITAL_INPUT_H_ */
