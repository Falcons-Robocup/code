// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef CONF_CLOCK_H_INCLUDED
#define CONF_CLOCK_H_INCLUDED

#define USE_EXTERNAL_OSCILLATOR
#ifdef USE_EXTERNAL_OSCILLATOR
// Added by Andre Pool to select the external oscillator to get a more accurate cpu clock
// (which will result in a more accurate velocity value)
// checkout AVR1003 xmega clock system
// use external oscillator is 16MHz
#define BOARD_XOSC_TYPE	XOSC_TYPE_XTAL
#define BOARD_XOSC_HZ	16000000UL
// for the 16MHz external oscillator use range 12 to 16MHz
#define CONFIG_XOSC_RANGE XOSC_RANGE_12TO16

// give the external oscillator some time to startup
#define BOARD_XOSC_STARTUP_US  1024

// use the external oscillator as source for the pll
#define CONFIG_PLL0_SOURCE	PLL_SRC_XOSC

// desire 128MHz as output frequency of the pll
#define BOARD_PLL_HZ	128000000UL

// pll frequency = oscillator * pllMultiply / pllDivide = 128MHz
#define CONFIG_PLL0_MUL	(BOARD_PLL_HZ / BOARD_XOSC_HZ)
#define CONFIG_PLL0_DIV	1

// cpu clock = pll fequency / ( prescaler A * prescaler B * prescaler C ) = 32MHz
#define CONFIG_SYSCLK_PSADIV	SYSCLK_PSADIV_1
#define CONFIG_SYSCLK_PSBCDIV	SYSCLK_PSBCDIV_2_2

// use the pll clock as clock fo the cpu
#define CONFIG_SYSCLK_SOURCE	SYSCLK_SRC_PLL

#else
//#define CONFIG_SYSCLK_SOURCE          SYSCLK_SRC_RC2MHZ
// Andre Pool: changed default system clock from the internal 2MHz to internal 32MHz oscillator
#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_RC32MHZ
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_RC32KHZ
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_XOSC
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_PLL

/* Fbus = Fsys / (2 ^ BUS_div) */
#define CONFIG_SYSCLK_PSADIV          SYSCLK_PSADIV_1
#define CONFIG_SYSCLK_PSBCDIV         SYSCLK_PSBCDIV_1_1

//#define CONFIG_PLL0_SOURCE          PLL_SRC_XOSC
//#define CONFIG_PLL0_SOURCE          PLL_SRC_RC2MHZ
//#define CONFIG_PLL0_SOURCE          PLL_SRC_RC32MHZ
//#define CONFIG_PLL0_SOURCE          PLL_SRC_XOSC

/* Fpll = (Fclk * PLL_mul) / PLL_div */
//#define CONFIG_PLL0_MUL             (24000000UL / BOARD_XOSC_HZ)
//#define CONFIG_PLL0_DIV             1

#endif

#endif /* CONF_CLOCK_H_INCLUDED */
