// Copyright 2015 Erik Kouters & Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_PID_H
#define INCLUDED_PID_H

#include <stdbool.h>

typedef struct
{
	uint16_t p;
	uint16_t i;
	uint16_t d;
	uint16_t iTh;
} pidPropT;

typedef struct {
	int16_t velocity;
	int16_t pidError;
	int32_t integral;
	int32_t result32;
} pidExportT;

void initPid();

// task is executed on a regular base
void taskPid();

void setAngleSetPoint( int16_t value );
void setPrimarySetPoint( int16_t value );
void setAngleDirection( bool value );
int16_t getAngleSetPoint();
int16_t getPrimarySetPoint();
bool getAngleDirection();

void setMode( uint8_t value );
uint8_t getMode();

void setPidAngleProperties(pidPropT value);
void setPidPrimaryProperties(pidPropT value);
pidPropT getPidAngleProperties();
pidPropT getPidPrimaryProperties();
pidExportT getPidPrimaryExport();
int16_t getPidAngleError();
int16_t getPidPrimaryError();
int32_t getPidAngleIntegral();
int32_t getPidPrimaryIntegral();
int16_t getPidPrimaryResult16();
int32_t getPidPrimaryResult32(); // only used for diagnostics (send to linux pc)

uint16_t getPidError();
void clearPidError(uint16_t value);

typedef struct {
	int32_t integral;
	bool direction; // only for angle, set relation between angle increase/decrease to clockwise/anti-clockwise increase/decrease
	int16_t derivative;
	pidPropT pidProperties;
	int16_t previousError;
	int16_t result16;
	int32_t result32; // only used for diagnostics (send to linux pc)
	int16_t setPoint;
} pidStateT;

#endif /* INCLUDED_PID_H */
