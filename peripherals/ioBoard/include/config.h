/*
 * config.h
 *
 *  Created on: Jun 8, 2017
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/wdt.h>

//! Defines the CPU frequency.
#define F_CPU	(16000000)	// 16 MHz

//! Defines whether the watchdog timer is enabled.
#define WATCHDOG_TIMER_ENABLED	   (1)

//! Defines the watchdog timer period.
#define WATCHDOG_TIMER_PERIOD	   (WDTO_1S)	// 1 sec

#define VERSION	("00000003")

#define PACKAGE_PAYLOAD_LENGTH     (8)

#define SHOOT_CHARGE_PORT	          PORTE
#define SHOOT_CHARGE_PIN	          (PIN0_bm)	// enable pin PE5

#define SHOOT_TRIGGER_PORT	          PORTE
#define SHOOT_TRIGGER_PIN	          (PIN1_bm)	// trigger pin PE6

#define SHOOT_LEVEL_PORT	          PORTE
#define SHOOT_LEVEL_DIRECTION_PIN         (PIN3_bm)
#define SHOOT_LEVEL_PWM_PIN	          (PIN4_bm)

#define KEEPER_FRAME_TRIGGER_PORT         PORTF        
#define KEEPER_FRAME_TRIGGER_RIGHT_PIN    (PIN2_bm)     
#define KEEPER_FRAME_TRIGGER_UP_PIN       (PIN3_bm)    
#define KEEPER_FRAME_TRIGGER_LEFT_PIN     (PIN4_bm)    

#define INOUT_PLAY_pm 0x40			// in or out of play pin A
#define INOUT_PLAY_shr 6

// Software pin
#define SOFTWARE_ON_PORT	PORTD
#define SOFTWARE_ON_PIN		PIN0_bm

#define RS422_TX_PORT		PORTC
#define RS422_TX_PIN		PIN3_bm
#define RS422_RX_PORT		PORTC
#define RS422_RX_PIN		PIN2_bm
#define RS422_ENABLE_PORT	PORTC
#define RS422_ENABLE_PIN	PIN4_bm

// Fixed command values from host
#define CMD_SET_SHOOT		      0x02
#define CMD_SET_HOME		      0x05
#define CMD_SET_LEVER_SPEED	      0x06
#define CMD_SET_HEIGHT		      0x07
#define CMD_GET_STATUS		      0x08
#define CMD_GET_VERSION		      0x56
#define CMD_SET_BOOTLOADER	      0xAA
#define CMD_SET_KEEPER_FRAME_RIGHT    0x09 
#define CMD_SET_KEEPER_FRAME_UP       0x0A 
#define CMD_SET_KEEPER_FRAME_LEFT     0x0B 
#define CMD_ERROR		      0xFF

#endif /* INCLUDE_CONFIG_H_ */
