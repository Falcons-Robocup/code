// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_SAFETY_H
#define INCLUDED_SAFETY_H

typedef struct {
	uint16_t tacho;
	uint16_t angle;
	uint16_t currentA;
	uint16_t currentB;
	uint16_t boardVoltage;
	uint16_t boardTemp;
} safetyT;

typedef struct {
	uint16_t tacho;
	uint16_t angle;
} angleTachoT;

void initSafety();

/*! \brief safety check that runs on a regular base to check for error conditions
 */
void taskSafety();

safetyT getSafety();

void validPacketRecieved();

void setMotorTimeout( uint16_t value );
uint16_t getMotorTimeout();

angleTachoT getAngleTachoZero();

/*! \brief return error code
 *
 * \retval multiple errors can occur, one error is using one bit, a maximum of 32 safety errors can be used
 * \retval return 0 if no error
 */
uint16_t getSafetyError();

/*! \brief clear error code(s)
 *
 * \param error bits that need to be cleared (e.g. when they have been send to pc)
 */
void clearSafetyError( uint16_t error );

#endif /* INCLUDED_SAFETY_H */
