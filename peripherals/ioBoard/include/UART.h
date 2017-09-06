/*
 *	Module		: UART.h
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 01-07-2013
 *  Description	: Controls the UART for the RS422 interface. It is using 
 *				  interrupts for a fast timing.   
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		:
 *
 */

#ifndef UART_H_
#define UART_H_

#include <stdbool.h>

void UART_initalize(void);
void UART_terminate(void);
void UART_putchar(char character);
char UART_getchar(void);
bool UART_has_data(void);

#endif /* UART_H_ */
