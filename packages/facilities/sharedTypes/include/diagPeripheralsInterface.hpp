// Copyright 2018-2020 Jan Feitsma (Falcons)
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
    float feedback_vel[3] = {0.0, 0.0, 0.0}; // measured speed
    float speed_vel[3] = {0.0, 0.0, 0.0}; // intended speed
    bool  hasball = false;
    float bh_left_angle = 0.0;
    float bh_right_angle = 0.0;
    float voltage = 0.0;
    float motion_temperature[3] = {0.0, 0.0, 0.0};
    float motor_pid_output[3] = {0.0, 0.0, 0.0};
    float motor_error[3] = {0.0, 0.0, 0.0};
    float motor_integral[3] = {0.0, 0.0, 0.0};
    float motor_derivative[3] = {0.0, 0.0, 0.0};

    SERIALIZE_DATA(feedback_vel, speed_vel, hasball, bh_left_angle, bh_right_angle, voltage, motion_temperature, motor_pid_output, motor_error, motor_integral, motor_derivative);
};

#endif

