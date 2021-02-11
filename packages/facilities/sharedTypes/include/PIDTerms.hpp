// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef PIDTERMS_HPP_
#define PIDTERMS_HPP_


#include "RtDB2.h" // required for serialization


struct PIDTerms
{
    // contributions to the end result, so including the parameters
    float proportional = 0.0;
    float integral = 0.0;
    float derivative = 0.0;

    SERIALIZE_DATA(proportional, integral, derivative);
};


#endif

