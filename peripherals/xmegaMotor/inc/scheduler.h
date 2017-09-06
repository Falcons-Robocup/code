// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_SCHEDULER_H
#define INCLUDED_SCHEDULER_H

typedef struct
{
	uint32_t measure;
	uint32_t calculation;
} schedulerTimeT;

/*! \brief configure the timers and interrupts used by the scheduler
 */
void initScheduler();

/*! \brief get error code of scheduler
 *
 * \retval every bit in the error value represents a unique scheduler error
 * \retval return 0 if no error
 */
uint16_t getSchedulerError();

/*! \brief clear one or more errors bits of the scheduler
 *
 * \param the bits that need to be cleared
 * if argument is 0 then no error is cleared
 */
void clearSchedulerError( uint16_t error );

/*! \brief get amount of 32MHz clock cycles during measurement and calculation
 *
 */
schedulerTimeT getSchedulerTime( );

#endif /* INCLUDED_SCHEDULER_H */
