/*
 *	Module		: UART.c
 *  System		: Turtle 5K I/O Board 
 *  Version		: V0.2
 *	Date		: 01-07-2013
 *  Description	: Controls the UART for the RS422 interface. It is using 
 *				  interrupts for a fast timing.   
 *  Author		: Dirk-Jan Vethaak
 *
 *	History		: added double buffer. buffer is only passed trough if sof , address and eof is correct. 
 *
 */

#include "config.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "UART.h"

char rx_buffer[256];
volatile uint8_t rx_buffer_read_index = 0;
volatile uint8_t rx_buffer_write_index = 0;

//! Initializes the UART communication.
//! \details	Initialization of UART for full duplex communication with baudrate 115200.
//! \author		Edwin Schreuder
void UART_initialize()
{
	// Configure TX/PC3 for output, RX/PC2 for input.
	RS422_TX_PORT.DIRSET = RS422_TX_PIN;

	// Set baud rate to 115200 with fPER = 16 MHz. Bsel = 983 and bscale = -7 0b1001.
	USARTC0.BAUDCTRLA = 0xD7; // Bsel  low byte
	USARTC0.BAUDCTRLB = 0x93; // Bscale nibble & Bsel high nibble

	// Clear all status flags.
	USARTC0.STATUS = 0x00;

	// Initialize the UART settings.
	USARTC0.CTRLA = 0x30; // Enable USARTE0 interrupts RX, high priority.
	USARTC0.CTRLB = 0x18; // Enable both transmitter and receiver.
	USARTC0.CTRLC = 0x03; // Select asynchronous USART, disable parity, 1 stop bit, 8 data bits.

	// TX enable of UART chip driver
	RS422_ENABLE_PORT.DIRSET = RS422_ENABLE_PIN; // PC4
	RS422_ENABLE_PORT.OUTSET = RS422_ENABLE_PIN; // TX enable of RS422 driver is set.
}

//! Initializes the UART communication.
//! \details	Initialization of UART for full duplex communication with baudrate 115200.
//! \author		Edwin Schreuder
void UART_terminate()
{
	// Disable the transceiver (high-impedance pin).
	RS422_ENABLE_PORT.DIRCLR = RS422_ENABLE_PIN; // PC4
	RS422_TX_PORT.DIRCLR = RS422_TX_PIN;

	// Clear the CTRB register to disable the transceiver.
	USARTC0.CTRLB = 0x00;
}

//! Sends a single character on the UART (blocking).
//! \author		Edwin Schreuder
void UART_putchar(char character)
{
	// Wait for the UART to be cleared.
	while(!(USARTC0.STATUS & USART_DREIF_bm)) {
	}

	// Put the character to be sent in the buffer.
	USARTC0.DATA = character;
}

//! Receives a character from the UART (blocking).
//! \author		Edwin Schreuder
char UART_getchar(void)
{
	// Wait for a character to be received.
	while(!UART_has_data()) {
	}

	char character = rx_buffer[rx_buffer_read_index];
	rx_buffer_read_index++;

	return character;
}

//! Returns whether the UART has a character available.
//! \author		Edwin Schreuder
bool UART_has_data()
{
	return rx_buffer_read_index != rx_buffer_write_index;
}

//! Interrupt service routine when receive buffer is full.
//! \details	The interrupt routine is fired when a byte is received by the UART.
//!				The byte will be put in a circular buffer.
//! \author		Edwin Schreuder
ISR(USARTC0_RXC_vect){
	// Read the data from the buffer; this will automatically clear the interrupt.
	uint8_t character = USARTC0.DATA;

	// Read the data in the read buffer. If the buffer is full, we need to discard it.
	if (rx_buffer_read_index != (rx_buffer_write_index + 1)) {
		rx_buffer[rx_buffer_write_index] = character;
		rx_buffer_write_index++;
	}
}
