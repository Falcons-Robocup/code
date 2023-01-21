// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocityControl.hpp
 *
 *  Created on: Oct, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROL_HPP_
#define VELOCITYCONTROL_HPP_


#include <vector>

// common Falcons headers
#include "ConfigInterface.hpp"

// VelocityControl interfaces
#include "int/InputInterface.hpp"
#include "int/OutputInterface.hpp"
#include "int/AbstractVelocitySetpointController.hpp"

// data struct
#include "int/VelocityControlData.hpp"

// use a short alias
using vcCFI = ConfigInterface<ConfigVelocityControl>;
using ppCFI = ConfigInterface<ConfigPathPlanning>;
using exCFI = ConfigInterface<ConfigExecution>;


class VelocityControl
{
public:
    VelocityControl(vcCFI *vcConfigInterface = NULL, ppCFI *ppConfigInterface = NULL, exCFI *exConfigInterface = NULL, InputInterface *inputInterface = NULL, OutputInterface *outputInterface = NULL);
    ~VelocityControl();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    void iterate();

    // raw calculation based on inputs, useful for unit testing
    void calculate();

public:
    // having these public is convenient for test suite
    VelocityControlData data;
    void prepare();
    void setOutputs();
    void clearVelocitySetpointController();

private:
    // helper functions
    void getInputs();
    diagVelocityControl makeDiagnostics();
    void setupVelocitySetpointController();
    AbstractVelocitySetpointController *getVelocitySetpointController();
    AbstractVelocitySetpointController *setupAndGetVelocitySetpointController();

    vcCFI                     *_vcConfigInterface;
    ppCFI                     *_ppConfigInterface;
    exCFI                     *_exConfigInterface;
    InputInterface            *_inputInterface;
    OutputInterface           *_outputInterface;
    AbstractVelocitySetpointController *_velocitySetpointController = NULL;
    VelocitySetpointControllerTypeEnum  _currentVelocitySetpointControllerType;

};

#endif

