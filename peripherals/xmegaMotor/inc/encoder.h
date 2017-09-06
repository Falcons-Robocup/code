// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_ENCODER_H
#define INCLUDED_ENCODER_H

#include <stdbool.h>
#include "global.h"
// functions to provide the encoder data

typedef struct
{
	int16_t velocity;
	int32_t displacement;
} encDataT;

void initEncoder();

void taskEncoder(bool measure);

encDataT getEncoderData();

uint16_t getEncoderError();
void clearEncoderError( uint16_t value );


#endif /* INCLUDED_ENCODER_H */
