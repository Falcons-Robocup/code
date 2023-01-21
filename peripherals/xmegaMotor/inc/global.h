// Copyright 2015, 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_GLOBAL_H
#define INCLUDED_GLOBAL_H

#include "pid.h"
#include "genericFilter2nd.h"

#define HARDWARE_VERSION 0x0002
#define SOFTWARE_VERSION 0x0018

#define DEVICE_ID 0x0042
#define VENDOR_ID 0x0009

// errors for the communication functions
// each bit represents a specific communication error
#define COMMUNICATION_ERROR_FROM_BOARD_WRITE               0x0001
#define COMMUNICATION_ERROR_FROM_BOARD_OUT_OF_RANGE        0x0002
#define COMMUNICATION_ERROR_FROM_BOARD_END_CHECK           0x0004
#define COMMUNICATION_ERROR_FROM_BOARD_SOF_OVERWRITTEN     0x0008
#define COMMUNICATION_ERROR_TO_BOARD_SIZE_OVERFLOW         0x0010
#define COMMUNICATION_ERROR_TO_BOARD_OVERFLOW              0x0020
#define COMMUNICATION_ERROR_TO_BOARD_CHECKSUM              0x0040
#define COMMUNICATION_ERROR_TO_BOARD_END_CHECK             0x0080
#define COMMUNICATION_ERROR_TO_BOARD_SOF_ALIGNMENT         0x0100
#define COMMUNICATION_ERROR_TO_BOARD_ILLEGAL_COMMAND       0x0200
#define COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC    0x0400
#define COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN    0x0800
#define COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW       0x1000
#define COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE           0x2000

#define ENCODER_ERROR_INIT_NOT_PERFORMED                   0x0001
#define ENCODER_ERROR_VELOCITY_MAX_NEGATIVE                0x0002
#define ENCODER_ERROR_VELOCITY_MAX_POSITIVE                0x0004

#define PID_ERROR_INIT_NOT_PERFORMED                 0x0001
#define PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE 0x0002
#define PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE      0x0004 // not used anymore
#define PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE    0x0008
#define PID_ERROR_PRIMARY_RESULT_OUT_OF_RANGE        0x0010
#define PID_ERROR_ANGLE_CURRENT_ERROR_OUT_OF_RANGE   0x0020
#define PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE        0x0040 // not used anymore
#define PID_ERROR_ANGLE_DERIVATIVE_OUT_OF_RANGE      0x0080
#define PID_ERROR_ANGLE_RESULT_OUT_OF_RANGE          0x0100
#define PID_ERROR_REQUEST_CLEAR_STATE                0x8000

#define SAFETY_ERROR_INIT_NOT_PERFORMED              0x0001
#define SAFETY_COMMUNICATION_TIMEOUT                 0x0002
#define SAFETY_ANGLE_TACHO_UNINITIALZED              0x0004
#define SAFETY_ANGLE_VALUE_TOO_HIGH                  0x0008
#define SAFETY_ANGLE_VALUE_TOO_LOW                   0x0010
#define SAFETY_TACHO_VALUE_TOO_HIGH                  0x0020
#define SAFETY_TACHO_VALUE_TOO_LOW                   0x0040

#define SCHEDULER_ERROR_INIT_NOT_PERFORMED           0x0001
#define SCHEDULER_ERROR_MEASURE_TIME_EXPIRED         0x0002
#define SCHEDULER_ERROR_CALCULATE_TIME_EXPIRED       0x0004
#define SCHEDULER_ERROR_TEST                         0x8000

#define PWM_ERROR_INIT_NOT_PERFORMED                 0x0001
#define PWM_ERROR_POSITIVE_LIMIT                     0x0002
#define PWM_ERROR_NEGATIVE_LIMIT                     0x0004
#define PWM_ERROR_POSITIVE_DELTA                     0x0008 // not used anymore
#define PWM_ERROR_NEGATIVE_DELTA                     0x0010 // not used anymore
#define PWM_ERROR_REQUEST_CLEAR_STATE                0x8000

#define DRV8301_ERROR_INIT_NOT_PERFORMED             0x0001
#define DRV8301_ERROR_FETLC_OC                       0x0002
#define DRV8301_ERROR_FETHC_OC                       0x0004
#define DRV8301_ERROR_FETLB_OC                       0x0008
#define DRV8301_ERROR_FETHB_OC                       0x0010
#define DRV8301_ERROR_FETLA_OC                       0x0020
#define DRV8301_ERROR_FETHA_OC                       0x0040
#define DRV8301_ERROR_OTW                            0x0080
#define DRV8301_ERROR_OTSD                           0x0100
#define DRV8301_ERROR_PVDD_UV                        0x0200
#define DRV8301_ERROR_GVDD_UV                        0x0400
#define DRV8301_ERROR_FAULT                          0x0800
#define DRV8301_ERROR_DEVICE_ID                      0x1000
#define DRV8301_ERROR_GVDD_OV                        0x2000
#define DRV8301_ERROR_CONTROL_REGISTER1              0x4000
#define DRV8301_ERROR_CONTROL_REGISTER2              0x8000

//
// Global variables
//
extern pidStateT anglePID; // only used in ball hander mode
extern pidStateT primaryPID; // used for wheel motor and ball handler tacho
extern genericFilter2ndValues angleLowPass;
extern genericFilter2ndValues primaryLowPass;

#endif /* INCLUDED_GLOBAL_H */
