// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_PWM_H
#define INCLUDED_PWM_H

#include <stdbool.h>

void initPwm();

// the pwm task should run on a fixed rate with a low jitter to improve the pid controller performance
void taskPwm();

int16_t getPwmValue();

void setPwmLimit(uint16_t value);
uint16_t getPwmLimit();

void setPwmDelta(uint16_t value);
uint16_t getPwmDelta();

uint16_t getPwmError();
void clearPwmError(uint16_t value);

#endif /* INCLUDED_PWM_H */
