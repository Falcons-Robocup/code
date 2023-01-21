// Copyright 2018-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagPeripheralsInterface.hpp
 *
 *  Created on: Nov 29, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef DIAGPERIPHERALSINTERFACE_HPP_
#define DIAGPERIPHERALSINTERFACE_HPP_

#include "RtDB2.h" // required for serialization
#include <string>

struct diagPeripheralsInterface
{
    // drive motors
    float current_feedback[3] = {0.0, 0.0, 0.0}; // measured current
    float current_setpoint[3] = {0.0, 0.0, 0.0}; // intended current
    float torque_feedback[3] = {0.0, 0.0, 0.0}; // measured torque
    float velocity_feedback[3] = {0.0, 0.0, 0.0}; // measured speed
    float velocity_setpoint[3] = {0.0, 0.0, 0.0}; // intended speed
    float motion_temperature[3] = {0.0, 0.0, 0.0};
    float voltage = 0.0;
    float motor_pid_output[3] = {0.0, 0.0, 0.0};
    float motor_error[3] = {0.0, 0.0, 0.0};
    float motor_integral[3] = {0.0, 0.0, 0.0};
    float motor_derivative[3] = {0.0, 0.0, 0.0};

    // ballhandlers
    float bh_angle[2] = {0.0, 0.0};
    float bh_tacho_zero[2] = {0.0, 0.0};
    float bh_tacho[2] = {0.0, 0.0};
    float bh_pid_output[2] = {0.0, 0.0};
    float bh_error[2] = {0.0, 0.0};
    float bh_integral[2] = {0.0, 0.0};
    float bh_pwm[2] = {0.0, 0.0};

    SERIALIZE_DATA(current_feedback, current_setpoint, torque_feedback, velocity_feedback, velocity_setpoint, motion_temperature, voltage, motor_pid_output, motor_error, motor_integral, motor_derivative, \
                    bh_angle, bh_tacho_zero, bh_tacho, bh_pid_output, bh_error, bh_integral, bh_pwm);
};

#endif

