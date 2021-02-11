// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanning.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_HPP_
#define PATHPLANNING_HPP_


#include <vector>

// common Falcons headers
#include "ConfigInterface.hpp"

// PathPlanning interfaces
#include "int/InputInterface.hpp"
#include "int/OutputInterface.hpp"

// data struct
#include "int/PathPlanningData.hpp"

// use a short alias
using CFI = ConfigInterface<ConfigPathPlanning>;


class PathPlanning
{
public:
    PathPlanning(CFI *configInterface = NULL, InputInterface *inputInterface = NULL, OutputInterface *outputInterface = NULL);
    ~PathPlanning();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    actionResultTypeEnum iterate();

    // raw calculation based on inputs, useful for unit testing
    actionResultTypeEnum calculate();

    // TODO: add an interface to provide (part of) the configuration
    // example use case: interceptBall calculation needs robot speed capability (maxVelXY)


public:
    // having these public is convenient for test suite
    PathPlanningData data;
    void prepare();
    void setOutputs();

private:
    // helper functions
    void getInputs();
    diagPathPlanning makeDiagnostics();

    CFI                       *_configInterface;
    InputInterface            *_inputInterface;
    OutputInterface           *_outputInterface;

};

#endif

