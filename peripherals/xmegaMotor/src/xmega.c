// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include "global.h"
#include "xmega.h"

#include "ioport.h"
#include "ioport_compat.h"
#include "sysclk.h"
#include "usart.h"
#include "status_codes.h"
#include "communication.h"

static uint8_t applicationId = 0;
static bool ledGreen = false;
static bool ledYellow = false;

static uint8_t rxBuffer[256] = {0,};
static uint8_t rxBufferWritePointer = 0;
static uint8_t rxBufferReadPointer = 0;

// minimize overhead when byte received to minimize jirter on encoder/pid/pwm cycle
ISR(USARTF0_RXC_vect)
{
	cli(); // disable global interrupt to be atomic
	// new byte available on rx port, store in rx buffer and increment write pointer (with wrap around)
	rxBuffer[rxBufferWritePointer] = usart_getchar(&USARTF0);
	rxBufferWritePointer++;
	if( rxBufferWritePointer == rxBufferReadPointer ) { setRxBufferOverflowError(); }
	sei(); // enable global interrupt again
}

void initXmega(){
	// first read the application ID from port e and port f, then set the application id pins
	// to output to measure the scheduler times
	// application ID0 = RJ45:2 = Enable = pe5
	// application ID1 = RJ45:6 = tx- = pe6
	// application ID2 = RJ45:5 = rx- = pe7
	// the selection application ID set through resistors to ground so pull ups are required
	PORTE.PIN5CTRL = PORT_OPC_PULLUP_gc; // enable pull up application ID0
	PORTE.PIN6CTRL = PORT_OPC_PULLUP_gc; // enable pull up application ID1
	PORTE.PIN7CTRL = PORT_OPC_PULLUP_gc; // enable pull up application ID2
	// reading the pins is done later in this function so the pull-up value can stabilize

	// port f is configured with direct register access instead of inport_configure functions
	// checkout /opt/avr_v3.4.5/avr/include/avr/iox64a3.h for bit definitions
	// configure drv8301 gate enable and led pins
	//      7      6     5        4        3      2        1      0
	//  scheduler  nc ledGreen ledYellow uartTx uartRx enableGate nc
	PORTF.DIR = 0xba; // scheduler, ledGreen, ledYellow, uartTX and enableGate as outputs

	// When the usb serial convert is disconnected from the pc, the converter generates noise that can
	// result in a "valid" packet. This can be prevented by using the pull up in the input of the xmega
	// see http://www.atmel.com/Images/doc8077.pdf chapter 13.13.14 PINnCTRL - Pin n Configuration register
	PORTF.PIN2CTRL = PORT_OPC_PULLUP_gc; // enable pull up on board rx to suppress noise

	setEnableGate(true); // enable gate of drv8301 (required for the spi interface)

	// set the used IO ports to the correct state
	ioport_init(); // this xmega function Initializes the IOPORT service, it needs to be called before usage of functions from IOPORT service

	ioport_configure_pin(PWM_A, IOPORT_DIR_OUTPUT);
	ioport_configure_pin(PWM_B, IOPORT_DIR_OUTPUT);

	// generate interrupt when one of the encoder input pins changes
	// e.g. PCICR |= ( 1<<PCIE1 );

	sysclk_init(); // checkout AVR1003 Xmega Clock System
	pmic_init(); // required for the uart ISR and ADC interrupt
	cpu_irq_enable();

	// read the application ID after a while so the pull up value has been settled
	// the three xmega applicationId pins are set to internal pull up
	// and the cable has a pull down on the applicationId pins that represent a one
	// So the value of the applicactionId pins is inverted to the applicationId value
	if( PORTE.IN & (1<<5) ) { applicationId |= (1<<0); }
	if( PORTE.IN & (1<<6) ) { applicationId |= (1<<1); }
	if( PORTE.IN & (1<<7) ) { applicationId |= (1<<2); }
	applicationId = 7 - applicationId;

	// port e is configured with direct register access instead of inport_configure functions
	// checkout /opt/avr_v3.4.5/avr/include/avr/iox64a3.h for bit definitions
	// configure applicationId pins as output for measuring the scheduler and enable pull up for the encoder
	//  7   6   5    4   3   2    1    0
	// id2 id1 id0  nc  nc  encI encB encA
	PORTE.DIR = 0xe0; // id2, id1 and id0 as outputs
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc; // enable pull up encA
	PORTE.PIN1CTRL = PORT_OPC_PULLUP_gc; // enable pull up encB
	PORTE.PIN2CTRL = PORT_OPC_PULLUP_gc; // enable pull up encI



	// configure serial interface to communicate with pc
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
			.baudrate = 115200, // TODO: for some reason higher baud rates do not work
			.charlength = USART_CHSIZE_8BIT_gc,
			.paritytype = USART_PMODE_DISABLED_gc,
			.stopbits = false
	};
	// sysclk_enable_module(SYSCLK_PORT_C, PR_USART0_bm); // TODO: what is the purpose of PRGEN in sysclk_enable_module?
	usart_init_rs232(&USARTF0, &USART_SERIAL_OPTIONS);
	usart_set_rx_interrupt_level(&USARTF0, USART_INT_LVL_LO);

#ifdef NONO
	// some low level port access which is already performed with the above statements
	USARTF0.CTRLB |= USART_RXEN_bm; // enalbe rx
	USARTF0.CTRLB |= USART_TXEN_bm; // enable tx

	static uint8_t sendData = 0;
	while( 1 ) {
		if (USARTF0.STATUS == USART_RXCIF_bm ) { setLed1(! getLed1()); }
	    sendByte(sendData);
	    sendData++;
	}
#endif

}

uint8_t getApplicationId(){
	return applicationId;
}

uint32_t getXmegaDeviceId(){
	// TODO: device returns only 3 bytes, add one byte at the msb location to create uint32_t
	// should return 0x42 0x96 0x1e
	return 0;
}

uint32_t getXmegaSerial(){
	// todo: do not know the return type of the serial number
	return 0;
}

uint32_t getCpuClkHz() {
	return sysclk_get_cpu_hz();
}

bool sendByte( uint8_t value ) {
	bool returnValue = ( usart_putchar(&USARTF0, value) == STATUS_OK );
	// enable the next 2 statements will result in performance penalty.
	// just fire and forget
	// while (!usart_tx_is_complete(&USARTF0)) { }
	// usart_clear_tx_complete(&USARTF0);
	return returnValue;
}

bool receiveByteAvailable() {
	return( rxBufferReadPointer != rxBufferWritePointer );
}

uint8_t receiveBufferSpace() {
	// the buffer is empty if read pointer points to the write pointer
	return( rxBufferReadPointer - rxBufferWritePointer - 1 );
}

uint8_t receiveByte() {
	uint8_t retVal = rxBuffer[rxBufferReadPointer];
	rxBufferReadPointer++;
	return retVal;
}

uint16_t getClockTicks() {
	// TODO: return value from 16 bits counter that is always running
	return 0;
}

void setLedGreen(bool value) {
	ledGreen = value;
	// use direct port access instead ioport_set_pin_level(LED_GREEN, ledGreen);
	// port f mapping
	//      7      6     5        4        3      2        1      0
	//  scheduler  nc ledGreen ledYellow uartTx uartRx enableGate nc
	if( value ) {
		PORTF.OUTSET = 0x20;
	} else {
		PORTF.OUTCLR = 0x20;
	}
}

bool getLedGreen() {
	return ledGreen;
}

void setLedYellow(bool value) {
	ledYellow = value;
	// use direct port access instead ioport_set_pin_level(LED_YELLOW, ledYellow);
	// port f mapping
	//      7      6     5        4        3      2        1      0
	//  scheduler  nc ledGreen ledYellow uartTx uartRx enableGate nc
	if( value ) {
		PORTF.OUTSET = 0x10;
	} else {
		PORTF.OUTCLR = 0x10;
	}
}

bool getLedYellow() {
	return ledYellow;
}

void setEnableGate( bool value ) {
	// use direct port access instead ioport_set_pin_level(EN_GATE, value);
	// port f mapping
	//      7      6     5        4        3      2        1      0
	//  scheduler  nc ledGreen ledYellow uartTx uartRx enableGate nc
	if( value ) {
		PORTF.OUTSET = 0x02; // enable drv8301 by asserting enable gate pin
	} else {
		PORTF.OUTCLR = 0x02; // disable drv8301 by deasserting enable gate pin
	}
}
