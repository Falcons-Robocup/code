// Copyright 2015 Jan Feitsma & Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_SPI_MASTER_H
#define INCLUDED_SPI_MASTER_H

#include "spi_master.h"

/*! \brief read from and write to the control registers of drv8301 motor driver controller
 *
 */

/* from iox64a3.h (= atmel chip type )
 */
#ifdef NONO
--------------------------------------------------------------------------
SPI - Serial Peripheral Interface
--------------------------------------------------------------------------
*/

/* Serial Peripheral Interface */
typedef struct SPI_struct
{
    register8_t CTRL;  /* Control Register */
    register8_t INTCTRL;  /* Interrupt Control Register */
    register8_t STATUS;  /* Status Register */
    register8_t DATA;  /* Data Register */
} SPI_t;

/* SPI Mode */
typedef enum SPI_MODE_enum
{
    SPI_MODE_0_gc = (0x00<<2),  /* SPI Mode 0 */
    SPI_MODE_1_gc = (0x01<<2),  /* SPI Mode 1 */
    SPI_MODE_2_gc = (0x02<<2),  /* SPI Mode 2 */
    SPI_MODE_3_gc = (0x03<<2),  /* SPI Mode 3 */
} SPI_MODE_t;

/* Prescaler setting */
typedef enum SPI_PRESCALER_enum
{
    SPI_PRESCALER_DIV4_gc = (0x00<<0),  /* System Clock / 4 */
    SPI_PRESCALER_DIV16_gc = (0x01<<0),  /* System Clock / 16 */
    SPI_PRESCALER_DIV64_gc = (0x02<<0),  /* System Clock / 64 */
    SPI_PRESCALER_DIV128_gc = (0x03<<0),  /* System Clock / 128 */
} SPI_PRESCALER_t;

/* Interrupt level */
typedef enum SPI_INTLVL_enum
{
    SPI_INTLVL_OFF_gc = (0x00<<0),  /* Interrupt Disabled */
    SPI_INTLVL_LO_gc = (0x01<<0),  /* Low Level */
    SPI_INTLVL_MED_gc = (0x02<<0),  /* Medium Level */
    SPI_INTLVL_HI_gc = (0x03<<0),  /* High Level */
} SPI_INTLVL_t;

...

/* SPI - Serial Peripheral Interface */
/* SPI.CTRL  bit masks and bit positions */
#define SPI_CLK2X_bm  0x80  /* Enable Double Speed bit mask. */
#define SPI_CLK2X_bp  7  /* Enable Double Speed bit position. */

#define SPI_ENABLE_bm  0x40  /* Enable Module bit mask. */
#define SPI_ENABLE_bp  6  /* Enable Module bit position. */

#define SPI_DORD_bm  0x20  /* Data Order Setting bit mask. */
#define SPI_DORD_bp  5  /* Data Order Setting bit position. */

#define SPI_MASTER_bm  0x10  /* Master Operation Enable bit mask. */
#define SPI_MASTER_bp  4  /* Master Operation Enable bit position. */

#define SPI_MODE_gm  0x0C  /* SPI Mode group mask. */
#define SPI_MODE_gp  2  /* SPI Mode group position. */
#define SPI_MODE0_bm  (1<<2)  /* SPI Mode bit 0 mask. */
#define SPI_MODE0_bp  2  /* SPI Mode bit 0 position. */
#define SPI_MODE1_bm  (1<<3)  /* SPI Mode bit 1 mask. */
#define SPI_MODE1_bp  3  /* SPI Mode bit 1 position. */

#define SPI_PRESCALER_gm  0x03  /* Prescaler group mask. */
#define SPI_PRESCALER_gp  0  /* Prescaler group position. */
#define SPI_PRESCALER0_bm  (1<<0)  /* Prescaler bit 0 mask. */
#define SPI_PRESCALER0_bp  0  /* Prescaler bit 0 position. */
#define SPI_PRESCALER1_bm  (1<<1)  /* Prescaler bit 1 mask. */
#define SPI_PRESCALER1_bp  1  /* Prescaler bit 1 position. */


/* SPI.INTCTRL  bit masks and bit positions */
#define SPI_INTLVL_gm  0x03  /* Interrupt level group mask. */
#define SPI_INTLVL_gp  0  /* Interrupt level group position. */
#define SPI_INTLVL0_bm  (1<<0)  /* Interrupt level bit 0 mask. */
#define SPI_INTLVL0_bp  0  /* Interrupt level bit 0 position. */
#define SPI_INTLVL1_bm  (1<<1)  /* Interrupt level bit 1 mask. */
#define SPI_INTLVL1_bp  1  /* Interrupt level bit 1 position. */


/* SPI.STATUS  bit masks and bit positions */
#define SPI_IF_bm  0x80  /* Interrupt Flag bit mask. */
#define SPI_IF_bp  7  /* Interrupt Flag bit position. */

#define SPI_WRCOL_bm  0x40  /* Write Collision bit mask. */
#define SPI_WRCOL_bp  6  /* Write Collision bit position. */

#endif

/* TODO: make sure endianness is OK (0 or 1) */
#define SPI_endianness (0)


/*! \brief initialize the drv8301
 */
void initSpiMaster();

/*! \brief drv8301 task that have to be performed on a regular base
 */
void taskSpiMaster();

typedef struct
{
	uint16_t status1; // status value from drv8301
	uint16_t status2;
	uint16_t getControl1; // control value from drv8301
	uint16_t getControl2;
	uint16_t setControl1; // last value written to drv8301
	uint16_t setControl2;
} drv8301Type;

// get the static local variables (which are updated on a regular basis) representing the status of the drv8301
drv8301Type getDrv8301( );

// update the control registers of the drv8301
void setDrv8301( uint16_t control1, uint16_t control2 );

uint16_t getSpiMasterError();
void clearSpiMasterError(uint16_t value);

#endif /* INCLUDED_SPI_MASTER_H */
